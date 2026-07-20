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
#pragma once

#include <cuda_runtime.h>
#include <gtest/gtest.h>
#include <thrust/device_vector.h>
#include <thrust/remove.h>

#include <algorithm>
#include <cuda/stream_ref>
#include <iostream>
#include <sophus/se3.hpp>
#include <string>

#include "odometry_cuda_utils/cuda_types.cuh"
#include "odometry_types/odometry_config.hpp"
#include "odometry_types/point_types.hpp"
#include "odometry_utils/utils.hpp"

// Helper function to check CUDA errors
#define CHECK_CUDA_ERROR()                                  \
  int deviceCount;                                          \
  cudaError_t error = cudaGetDeviceCount(&deviceCount);     \
  if (error != cudaSuccess || deviceCount == 0) {           \
    GTEST_SKIP() << "No CUDA devices found, skipping test"; \
  }

namespace tam::core::state::cuda::utils {
/**
 * @brief Transform points with a given transformation
 * @param [in] T                  Transformation
 * @param [in] points             Points to transform
 */
template <typename TConfig>
void __host__ transform_points(
  const Sophus::SE3f& T, thrust::device_vector<types::Point<TConfig>>& points, ::cuda::stream_ref stream = {})
{
  thrust::transform(thrust::cuda::par.on(stream.get()), points.begin(), points.end(), points.begin(),
    [T] __device__(const types::Point<TConfig>& point) {
      types::Point<TConfig> pt = point;
      pt.pos = T * point.pos;
      // Transform normals and covariances if they exist
      if constexpr (TConfig::NORMALS) {
        pt.normal = T.rotationMatrix() * point.normal;
      }
      if constexpr (TConfig::COV) {
        pt.cov = T.rotationMatrix() * point.cov * T.rotationMatrix().transpose();
      }
      return pt;
    });
}

/**
 * @brief Copy function for different point types
 * @param [in] src                  Point to convert to different format
 */
template <typename TConfigSrc, typename TConfigDst>
__device__ types::Point<TConfigDst> convert_point(const types::Point<TConfigSrc>& src)
{
  types::Point<TConfigDst> dst;
  // Unconditional members of types::Point.
  dst.pos = src.pos;
  dst.timestamp = src.timestamp;
  CONDITIONAL_COPY(SPHERICAL, azimuth);
  CONDITIONAL_COPY(SPHERICAL, range);
  CONDITIONAL_COPY(SPHERICAL, elevation);
  CONDITIONAL_COPY(INTENSITY, intensity);
  CONDITIONAL_COPY(NORMALS, normal);
  CONDITIONAL_COPY(COV, cov);
  CONDITIONAL_COPY(SEG, seg);
  CONDITIONAL_COPY(VEL, vel);
  CONDITIONAL_COPY(RCS, rcs);
  CONDITIONAL_COPY(SNR, snr);
  CONDITIONAL_COPY(CONFIDENCE, confidence);
  CONDITIONAL_COPY(VEL_INTERVAL, vel_interval);
  CONDITIONAL_COPY(SENSOR_ID, sensor_id);
  return dst;
}

/**
 * @brief Check if a matrix has NaN values
 * @param [in] mat Matrix to check
 * @return True if the matrix has NaN values, false otherwise
 */
template <int N, int M>
__inline__ __device__ bool hasNaN(const Eigen::Matrix<float, N, M>& mat)
{
#pragma unroll
  for (int i = 0; i < N; ++i) {
#pragma unroll
    for (int j = 0; j < M; ++j) {
      if (isnan(mat(i, j))) {
        return true;
      }
    }
  }
  return false;
}

/**
 * @brief Warp-level reduction for 42 floats
 * @param [in] vals 42 floats to reduce
 */
template <int BLOCK_PARTIAL_SIZE>
__inline__ __device__ void warp_reduce_sum(float* vals)
{
// Standard "shuffle-down" pattern
#pragma unroll
  for (int offset = WARP_SIZE / 2; offset > 0; offset >>= 1) {
#pragma unroll
    for (int i = 0; i < BLOCK_PARTIAL_SIZE; i++) {
      float tmp = __shfl_down_sync(0xffffffff, vals[i], offset);
      vals[i] += tmp;
    }
  }
}

/**
 * @brief Set the CUDA device and print its properties
 * @param [in] device_id ID of the device to set
 * @return Properties of the set device
 */
cudaDeviceProp __host__ set_device(int device_id)
{
  cudaError_t err = cudaSetDevice(device_id);
  if (err != cudaSuccess) {
    throw std::runtime_error(std::string("CUDA set device failed: ") + cudaGetErrorString(err));
  }
  cudaDeviceProp prop;
  cudaGetDeviceProperties(&prop, device_id);
  std::cout << "\033[1;33mDevice: " << prop.name << "!\033[0m" << std::endl;
  std::cout << "\033[1;33mMultiprocessors: " << prop.multiProcessorCount << "\033[0m" << std::endl;
  return prop;
}

/**
 * @brief Allocate a thrust device vector with error handling
 * @param [in] vec  Vector to allocate
 * @param [in] size Size of the vector
 */
template <typename T>
void __host__ allocate_vector(thrust::device_vector<T>& vec, size_t size)
{
  try {
    vec.reserve(size);
    vec.resize(size);
  } catch (const thrust::system_error& e) {
    std::cerr << "Thrust error: " << e.what() << std::endl;
  }
}
}  // namespace tam::core::state::cuda::utils
