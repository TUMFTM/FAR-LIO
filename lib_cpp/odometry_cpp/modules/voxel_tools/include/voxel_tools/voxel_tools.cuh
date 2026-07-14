/*
 * Copyright 2026 Maximilian Leitenstern, Marcel Weinmann
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */
// CUDA Implementation strongly based on
// https://github.com/PRBonn/kiss-icp/blob/main/cpp/kiss_icp/core/VoxelHashMap.hpp
#pragma once

#include <cmath>
#include <cuda_runtime.h>
#include <thrust/device_vector.h>
#include <thrust/host_vector.h>
#include <thrust/pair.h>
#include <thrust/sort.h>
#include <thrust/transform.h>
#include <thrust/unique.h>

#include <Eigen/Core>
#include <algorithm>
#include <cuda/stream_ref>
#include <tuple>
#include <vector>

#include "odometry_cuda_utils/cuda_types.cuh"
#include "odometry_types/point_types.hpp"
namespace tam::core::state::cuda
{
// Point definition as tam::core::state::type::Point
using tam::core::state::types::Point;
/**
 * @brief Convert a point to a voxel
 * @param [in] point        Point to convert
 * @param [in] voxel_size   Size of the voxel
 */
template <typename TConfig>
struct PointToVoxel
{
  float voxel_size;
  __host__ __device__ PointToVoxel(float size) : voxel_size(size) {}
  __host__ __device__ custom_voxel_type operator()(const Point<TConfig> & point) const
  {
    return custom_voxel_type(
      static_cast<int16_t>(floorf(point.pos.x() / voxel_size)),
      static_cast<int16_t>(floorf(point.pos.y() / voxel_size)),
      static_cast<int16_t>(floorf(point.pos.z() / voxel_size)));
  }
};
struct custom_key_equal
{
  __host__ __device__ bool operator()(
    custom_voxel_type const & lhs, custom_voxel_type const & rhs) const noexcept
  {
    return lhs.x == rhs.x && lhs.y == rhs.y && lhs.z == rhs.z && lhs.padding == rhs.padding;
  }
};
/**
 * @brief Find the voxel indicies of the nearest voxels in the hashmap
 * @param [in] voxel              Input voxel indice
 * @param [in] voxel_neighborhood Raw pointer to the neighborhood to be determined
 * @param [in] adjacent_voxels    Number of neighbors to be considered
 */
__device__ void get_adjacent_voxels(
  const custom_voxel_type & voxel, custom_voxel_type * voxel_neighborhood,
  const int16_t adjacent_voxels)
{
  int index = 0;
  for (int16_t i = voxel.x - adjacent_voxels; i <= voxel.x + adjacent_voxels; ++i) {
    for (int16_t j = voxel.y - adjacent_voxels; j <= voxel.y + adjacent_voxels; ++j) {
      for (int16_t k = voxel.z - adjacent_voxels; k <= voxel.z + adjacent_voxels; ++k) {
        voxel_neighborhood[index++] = custom_voxel_type(i, j, k);
      }
    }
  }
}
struct VoxelSort
{
  __host__ __device__ bool operator()(
    const custom_voxel_type & a, const custom_voxel_type & b) const
  {
    if (a.x != b.x) return a.x < b.x;
    if (a.y != b.y) return a.y < b.y;
    return a.z < b.z;
  }
};
/**
 * @brief Downsample a frame of points to a voxel grid
 * => Only one point per voxel
 * @param [in] frame        Frame of points to downsample
 * @param [in] voxel_size   Size of the voxel
 * @return Downsampled frame
 */
template <typename TConfig>
void __host__ voxel_downsample(
  thrust::device_vector<types::Point<TConfig>> & frame_in,
  thrust::device_vector<types::Point<TConfig>> & frame_out, const float voxel_size,
  const uint32_t preallocated_memory, ::cuda::stream_ref stream = {})
{
  // create vector with voxels
  thrust::device_vector<custom_voxel_type> voxel_grid(frame_in.size());

  // transform the point cloud into a voxel grid
  thrust::transform(
    thrust::cuda::par.on(stream.get()), frame_in.begin(), frame_in.end(), voxel_grid.begin(),
    PointToVoxel<TConfig>(voxel_size));

  // remove duplicated voxels
  thrust::sort_by_key(
    thrust::cuda::par.on(stream.get()), voxel_grid.begin(), voxel_grid.end(), frame_in.begin(),
    VoxelSort());
  auto new_end = thrust::unique_by_key(
    thrust::cuda::par.on(stream.get()), voxel_grid.begin(), voxel_grid.end(), frame_in.begin());

  // resize the frame device vector
  // and free the device memory if more than the preallocated memory is used
  frame_out.resize(thrust::distance(frame_in.begin(), new_end.second));
  thrust::transform(
    thrust::cuda::par.on(stream.get()), frame_in.begin(), new_end.second, frame_out.begin(),
    [] __device__(const types::Point<TConfig> & p) { return p; });
  if (frame_out.size() > preallocated_memory) frame_out.shrink_to_fit();
}
/**
 * @brief Downsample a frame of points to a voxel grid
 * @param [in] frame        Frame of points to downsample
 * @param [in] voxel_size   Size of the voxel
 * @return Downsampled frame
 */
template <typename TConfig>
std::vector<Point<TConfig>> __host__
voxel_downsample(const std::vector<Point<TConfig>> & frame, const float voxel_size)
{
  // create CUDA Stream
  cudaStream_t stream;
  cudaStreamCreate(&stream);

  // load the data to the GPU
  thrust::device_vector<Point<TConfig>> frame_device(frame.size());
  thrust::device_vector<Point<TConfig>> frame_device_out(frame.size());
  thrust::copy(frame.begin(), frame.end(), frame_device.begin());

  // downsample the point cloud
  voxel_downsample(frame_device, frame_device_out, voxel_size, frame.size(), stream);

  // synchronize the stream
  cudaStreamSynchronize(stream);
  cudaStreamDestroy(stream);

  // return the downsampled point cloud
  std::vector<Point<TConfig>> frame_downsampled(frame_device_out.size());
  thrust::copy(frame_device_out.begin(), frame_device_out.end(), frame_downsampled.begin());

  return frame_downsampled;
}
/**
 * @brief Downsample a frame of points to a voxel grid twice with different
 * voxel sizes
 * @param [in] frame        Frame of points to downsample
 * @param [in] voxel_size_1   Size of the voxel for first downsampling
 * @param [in] voxel_size_2   Size of the voxel for second downsmapling
 * @return Downsampled frame
 */
template <typename TConfig>
void __host__ voxel_doubledownsample_device(
  thrust::device_vector<types::Point<TConfig>> & frame,
  thrust::device_vector<types::Point<TConfig>> & frame_0_5,
  thrust::device_vector<types::Point<TConfig>> & frame_1_5, const float voxel_size,
  const uint32_t preallocated_memory, ::cuda::stream_ref stream = {})
{
  // first voxelization
  voxel_downsample(frame, frame_0_5, voxel_size * 0.5, preallocated_memory, stream);

  // second voxelization
  voxel_downsample(frame_0_5, frame_1_5, voxel_size * 1.5, preallocated_memory, stream);
}
template <typename TConfig>
std::tuple<std::vector<types::Point<TConfig>>, std::vector<types::Point<TConfig>>> __host__
voxel_doubledownsample(const std::vector<types::Point<TConfig>> & frame, const float voxel_size)
{
  // create CUDA Stream
  cudaStream_t stream;
  cudaStreamCreate(&stream);

  // create the device vectors
  thrust::device_vector<types::Point<TConfig>> frame_device(frame.size());
  thrust::copy(frame.begin(), frame.end(), frame_device.begin());
  thrust::device_vector<types::Point<TConfig>> frame_0_5;
  thrust::device_vector<types::Point<TConfig>> frame_1_5;

  // Call cuda function
  voxel_doubledownsample_device(
    frame_device, frame_0_5, frame_1_5, voxel_size, frame.size(), stream);

  // synchronize the stream
  cudaStreamSynchronize(stream);
  cudaStreamDestroy(stream);

  // Copy data back to host and return
  std::vector<types::Point<TConfig>> frame_1_5_host(frame_1_5.size());
  thrust::copy(frame_1_5.begin(), frame_1_5.end(), frame_1_5_host.begin());
  std::vector<types::Point<TConfig>> frame_0_5_host(frame_0_5.size());
  thrust::copy(frame_0_5.begin(), frame_0_5.end(), frame_0_5_host.begin());

  return std::make_tuple(frame_1_5_host, frame_0_5_host);
}
struct custom_hash
{
  __host__ __device__ uint32_t operator()(custom_voxel_type const & k) const noexcept
  {
    return static_cast<uint32_t>(
      static_cast<uint32_t>(k.x) * 73856093 ^ static_cast<uint32_t>(k.y) * 19349669 ^
      static_cast<uint32_t>(k.z) * 83492791);
  }
};
}  // namespace tam::core::state::cuda
