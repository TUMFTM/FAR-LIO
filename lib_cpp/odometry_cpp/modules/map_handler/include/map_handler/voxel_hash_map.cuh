/*
 * Copyright 2024 Maximilian Leitenstern, Marcel Weinmann
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
// Implementation strongly based on
// https://github.com/PRBonn/kiss-icp/blob/main/cpp/kiss_icp/core/VoxelHashMap.hpp
#pragma once

#include <cuda_runtime.h>
#include <nvtx3/nvToolsExt.h>
#include <thrust/device_vector.h>
#include <thrust/execution_policy.h>
#include <thrust/iterator/zip_iterator.h>
#include <thrust/remove.h>
#include <thrust/tuple.h>

#include <algorithm>
#include <atomic>
#include <cfloat>
#include <cmath>
#include <cstddef>
#include <cuco/static_map.cuh>
#include <cuco/static_set.cuh>
#include <cuda/std/atomic>
#include <iostream>
#include <limits>
#include <memory>
#include <sophus/se3.hpp>
#include <string>
#include <vector>

#include "map_handler/map_handler_base.hpp"
#include "voxel_tools/voxel_tools.cuh"

namespace tam::core::state::cuda {
/**
 * @brief Kernel to find the closest neighbors between the scan and the map
 * @param [in] map_ref            Reference to the Hashmap
 * @param [in] points             Raw pointer to the input point cloud
 * @param [in] correspondences    Raw pointer to the correspondence vector
 * @param [in] voxel_size         Voxel size for the input point cloud
 * @param [in] adjacent_voxels    Number of adjacent voxels to search in each direction
 * @param [in] num_points         Number of points in the input point cloud
 */
template <typename TConfig, typename Map>
__global__ void search_closest_neighbor_kernel(Map map_ref, types::Point<TConfig>* __restrict__ points,
  types::Correspondence<TConfig>* __restrict__ correspondences, const double voxel_size, const int16_t adjacent_voxels,
  const size_t num_points)
{
  auto tid = threadIdx.x + blockIdx.x * blockDim.x;
  int16_t num_neighbors = (2 * adjacent_voxels + 1) * (2 * adjacent_voxels + 1) * (2 * adjacent_voxels + 1);

  while (tid < num_points) {
    const auto query_point = points[tid];
    auto voxel = PointToVoxel<TConfig>(voxel_size)(query_point);

    // TODO(Maxi/Marcel): The allocation should "num_neighbors", however nvcc requires the size
    // to be known at compile time. This is a workaround for now.
    custom_voxel_type local_neighbors[27];
    get_adjacent_voxels(voxel, local_neighbors, adjacent_voxels);
    types::Point<TConfig> closest_neighbor = query_point;
    float closest_distance_sq = FLT_MAX;

    for (int16_t i = 0; i < num_neighbors; ++i) {
      auto found = map_ref.find(local_neighbors[i]);
      if (found != map_ref.end()) {
        // Capture pointer to the content of the voxel
        auto* voxel_data = found->second;
        for (int32_t j = 0; j < voxel_data->counter; ++j) {
          float distance_sq = (voxel_data->data[j].pos - query_point.pos).squaredNorm();
          if (distance_sq < closest_distance_sq) {
            closest_distance_sq = distance_sq;
            closest_neighbor = voxel_data->data[j];
          }
        }
      }
    }
    correspondences[tid].frame = points[tid];
    correspondences[tid].map = closest_neighbor;
    correspondences[tid].distance = std::sqrt(closest_distance_sq);
    tid += gridDim.x * blockDim.x;
  }
}

/**
 * @brief Kernel to find the closest neighbors between the scan and the map
 * @param [in] map_ref            Reference to the Hashmap
 * @param [in] neighbors          Raw pointer to the neighbors vector
 * @param [in] voxel_size         Voxel size for the input point cloud
 * @param [in] adjacent_voxels    Number of adjacent voxels to search in each direction
 * @param [in] num_neighbors      Number of neighbors to search for
 * @param [in] num_points         Number of points in the input point cloud
 * @note The found neighbors are not sorted by distance
 * @note Function does not check whether num_neighbors exceeds TConfig::NUM_NEIGHBORS
 */
template <typename TConfig, typename Map>
__global__ void search_closest_neighbors_kernel(Map map_ref, types::Neighbors<TConfig>* __restrict__ neighbors,
  const double voxel_size, const int16_t adjacent_voxels, const int16_t num_neighbors, const size_t num_points)
{
  auto tid = threadIdx.x + blockIdx.x * blockDim.x;
  int16_t num_neighbor_voxels = (2 * adjacent_voxels + 1) * (2 * adjacent_voxels + 1) * (2 * adjacent_voxels + 1);

  while (tid < num_points) {
    const auto query_point = *neighbors[tid].point;
    auto voxel = PointToVoxel<TConfig>(voxel_size)(query_point);

    // TODO(Maxi/Marcel): The allocation should "num_neighbor_voxels", however nvcc requires the
    // size to be known at compile time. This is a workaround for now.
    custom_voxel_type local_neighbors[27];
    get_adjacent_voxels(voxel, local_neighbors, adjacent_voxels);

    // Create local arrays to avoid repeated global memory writing
    float local_distances[TConfig::NUM_NEIGHBORS] = {};
    types::Point<TConfig>* local_neighbor_ptrs[TConfig::NUM_NEIGHBORS] = {};
    std::uint8_t current_neighbors = 0;

    for (int16_t i = 0; i < num_neighbor_voxels; ++i) {
      auto found = map_ref.find(local_neighbors[i]);
      if (found != map_ref.end()) {
        // Capture pointer to the content of the voxel
        auto* voxel_data = found->second;
        for (int32_t j = 0; j < voxel_data->counter; ++j) {
          const float distance_sq = (voxel_data->data[j].pos - query_point.pos).squaredNorm();
          // check if map is already full
          if (current_neighbors == num_neighbors) {
            float maximal_distance_sq = 0.0f;
            int16_t largest_neighbor_index = -1;
#pragma unroll
            // iterate through all existing neighbors
            for (int16_t neighbor_idx = 0; neighbor_idx < num_neighbors; ++neighbor_idx) {
              // if current distance is larger than maximal_distance_sq and distance
              // update
              if (maximal_distance_sq < local_distances[neighbor_idx] && distance_sq < local_distances[neighbor_idx]) {
                maximal_distance_sq = local_distances[neighbor_idx];
                largest_neighbor_index = neighbor_idx;
              }
            }
            // replace largest neighbor with current one
            if (largest_neighbor_index != -1) {
              local_neighbor_ptrs[largest_neighbor_index] = &voxel_data->data[j];
              local_distances[largest_neighbor_index] = distance_sq;
            }
          } else {
            // if map is not full just insert it into the neighbors
            local_distances[current_neighbors] = distance_sq;
            local_neighbor_ptrs[current_neighbors++] = &voxel_data->data[j];
          }
        }
      }
    }
    // Write neighbors to global memory
    neighbors[tid].num_neighbors = current_neighbors;
#pragma unroll
    for (int16_t i = 0; i < current_neighbors; ++i) {
      neighbors[tid].neighbor[i] = local_neighbor_ptrs[i];
      neighbors[tid].distance[i] = std::sqrt(local_distances[i]);
    }
    tid += gridDim.x * blockDim.x;
  }
}

/**
 * @brief Kernel to insert points into the voxel hashmap
 * @param [in] map_ref            Reference to the Hashmap
 * @param [in] points             Raw pointer to the points to insert in the map
 * @param [out] neighbors         Raw pointer to the neighbors vector storing the inserted points
 * @param [in] idx                Index to keep track of the number of points inserted
 * @param [in] num_points         Number of points to insert
 * @param [in] voxel_size         Voxel size for the hashmap
 * @param [in] map_resolution     Min distance between points within a voxel (m)
 * @param [in] max_points         Max points per voxel
 * @param [in] min_points         Min value for the adaptive max points per voxel
 * @param [in] range              Range around origin without adaptive max points (m); 0 disables
 * @param [in] max_points_scale   Change of max points per meter beyond range
 * @param [in] origin_x           Origin (x) for the adaptive max points
 * @param [in] origin_y           Origin (y) for the adaptive max points
 * @param [in] origin_z           Origin (z) for the adaptive max points
 */
template <typename TConfig, typename Map>
__global__ void insert_values_kernel(Map map_ref, const types::Point<types::Point_XYZ>* __restrict__ points,
  types::Neighbors<TConfig>* __restrict__ neighbors, unsigned int* idx, const size_t num_points,
  const double voxel_size, const double map_resolution, const int16_t max_points = TConfig::MAX_POINTS_PER_VOXEL,
  const int16_t min_points = TConfig::MAX_POINTS_PER_VOXEL, const float range = 0.0, const float max_points_scale = 0.0,
  const float origin_x = 0.0f, const float origin_y = 0.0f, const float origin_z = 0.0f)
{
  auto tid = threadIdx.x + blockIdx.x * blockDim.x;
  Eigen::Vector3f origin(origin_x, origin_y, origin_z);
  const bool adaptive_max_points = abs(range) > 1e-3;

  while (tid < num_points) {
    // Find the voxel of the point
    auto found = map_ref.find(PointToVoxel<types::Point_XYZ>(voxel_size)(points[tid]));

    if (found != map_ref.end()) {
      // Compute max number of points before acquiring the lock to avoid unnecessary waiting
      int16_t max_points_per_voxel = max_points;
      if (adaptive_max_points) {
        const float distance2origin = (points[tid].pos - origin).norm();
        if (distance2origin > range) {
          // clang-format off
          max_points_per_voxel = std::max(min_points, static_cast<int16_t>(max_points + (distance2origin - range) * max_points_scale));  // NOLINT
          // clang-format on
        }
      }
      int* lock = &found->second->lock;

      // Check if the voxel is already locked
      while (atomicCAS(lock, 0, 1) != 0) {
      }
      bool insert = true;
      // Check if the points is too close to another point in the voxel
      for (int32_t j = 0; j < found->second->counter; ++j) {
        if ((found->second->data[j].pos - points[tid].pos).norm() < map_resolution) {
          insert = false;
          break;
        }
      }
      // Insert the point if the voxel is not full
      if (found->second->counter < max_points_per_voxel && insert) {
        int32_t current_count = found->second->counter;
        found->second->data[found->second->counter] = utils::convert_point<types::Point_XYZ, TConfig>(points[tid]);
        ++found->second->counter;
        // Store pointer to the point in the map
        // Note: atomicAdd returns value before incrementing, so we can use it to get the index
        unsigned int local_idx = atomicAdd(idx, 1);
        neighbors[local_idx].point = &found->second->data[current_count];
      }
      atomicExch(lock, 0);
    }
    tid += gridDim.x * blockDim.x;
  }
}

/**
 * @brief Kernel to copy values from the active to the inactive hashmap
 * @param [in] map_active         Reference to the active Hashmap
 * @param [in] map_inactive       Reference to the inactive Hashmap
 * @param [in] keys               Raw pointer to the keys for the values to copy
 * @param [in] num_values         Number of values to copy
 */
template <typename TConfig, typename Map>
__global__ void copy_values_kernel(
  Map map_active, Map map_inactive, const custom_voxel_type* __restrict__ keys, const size_t num_values)
{
  int idx = blockIdx.x * blockDim.x + threadIdx.x;
  if (idx >= num_values) return;

  custom_voxel_type key = keys[idx];

  auto src_entry = map_active.find(key);
  auto dst_entry = map_inactive.find(key);

  if (src_entry == map_active.end() || dst_entry == map_inactive.end()) return;
  if (src_entry->second == nullptr || dst_entry->second == nullptr) return;

  auto* src = src_entry->second;
  auto* dst = dst_entry->second;

  // compute the number of bytes to copy (16-byte vector for coalesced access)
  using vec_t = uint4;
  int32_t count = src->counter;
  constexpr size_t header_size = offsetof(fixed_sized_points_array<TConfig>, data);
  constexpr size_t point_size = sizeof(Point<TConfig>);
  size_t bytes_to_copy = header_size + count * point_size;
  size_t n = bytes_to_copy / sizeof(vec_t);
  size_t r = bytes_to_copy % sizeof(vec_t);

  auto* dst_vec = reinterpret_cast<vec_t*>(dst);
  auto* src_vec = reinterpret_cast<const vec_t*>(src);

#pragma unroll
  for (size_t i = 0; i < n; ++i) {
    dst_vec[i] = src_vec[i];
  }

  if (r > 0) {
    auto* dst_bytes = reinterpret_cast<char*>(dst);
    auto* src_bytes = reinterpret_cast<const char*>(src);
#pragma unroll
    for (size_t i = bytes_to_copy - r; i < bytes_to_copy; ++i) {
      dst_bytes[i] = src_bytes[i];
    }
  }
}

template <typename TConfig>
class VoxelHashMap : public MapHandler<TConfig>
{
public:
  // Define map type
  using StaticMapType = cuco::static_map<custom_voxel_type, fixed_sized_points_array<TConfig>*,
    cuco::extent<std::size_t, types::MAX_MAP_SIZE>, ::cuda::thread_scope_device, custom_key_equal,
    cuco::linear_probing<1, custom_hash>>;

public:
  /**
   * @brief Constructor for param manager and logger
   */
  static std::unique_ptr<MapHandler<TConfig>> from_config(
    tam::pmg::ParamReferenceManager* pmg, tam::tsl::ReferenceLogger* logger)
  {
    std::unique_ptr<VoxelHashMap<TConfig>> mh =
      std::unique_ptr<VoxelHashMap<TConfig>>(new VoxelHashMap<TConfig>(pmg, logger));
    return mh;
  }

  /**
   * @brief Constructor for config and debug objects
   */
  static std::unique_ptr<MapHandler<TConfig>> from_config(const types::MapConfig& config, const types::MapDebug& debug)
  {
    std::unique_ptr<VoxelHashMap<TConfig>> mh =
      std::unique_ptr<VoxelHashMap<TConfig>>(new VoxelHashMap<TConfig>(config, debug));
    return mh;
  }

  /**
   * @brief Clear the map
   */
  __host__ void clear() override
  {
    map_a_.clear();
    map_b_.clear();
    this->debug_.num_voxel = 0;
    this->debug_.num_points = 0;
  }

  /**
   * @brief Check if map is empty
   */
  __host__ bool empty() const override { return this->get_active_map().size() == 0; }

  /**
   * @brief Get the amount of points in the map
   * @return                        Number of points in the map
   */
  __host__ std::size_t num_points(::cuda::stream_ref stream = {}) const override
  {
    nvtxRangePush("num_points");
    unsigned int h_counter = 0;
    cudaMemset(this->counter_d_, 0, sizeof(unsigned int));
    unsigned int* counter_ptr = this->counter_d_;

    this->get_active_map().for_each(
      [counter_ptr] __device__(auto const& slot) { atomicAdd(counter_ptr, slot.second->counter); }, stream.get());

    stream.wait();
    cudaMemcpy(&h_counter, this->counter_d_, sizeof(unsigned int), cudaMemcpyDeviceToHost);
    nvtxRangePop();
    return static_cast<std::size_t>(h_counter);
  }

  /**
   * @brief search for closest neighbor of a point
   * @param [in] point                       Point/ Points to search for
   * @param [in] correspondences             Empty vector to return the correspondences in
   * @param [in] adjacent_voxels             Number of adjacent voxels to search in
   * @param [in] stream                      Reference to the CUDA stream to use
   */
  __host__ void search_closest_neighbor(thrust::device_vector<types::Point<TConfig>>& points,
    thrust::device_vector<types::Correspondence<TConfig>>& correspondences, const int16_t adjacent_voxels = 1,
    ::cuda::stream_ref stream = {}) const override
  {
    nvtxRangePush("search_closest_neighbor");
    correspondences.resize(points.size());
    auto grid_size = std::min(4 * this->num_multiprocessors_, (points.size() + BLOCK_SIZE - 1) / BLOCK_SIZE);

    types::Point<TConfig>* raw_ptr_frame = thrust::raw_pointer_cast(points.data());
    types::Correspondence<TConfig>* raw_ptr_correspondences = thrust::raw_pointer_cast(correspondences.data());

    search_closest_neighbor_kernel<<<grid_size, BLOCK_SIZE, 0, stream.get()>>>(this->get_active_map().ref(cuco::find),
      raw_ptr_frame, raw_ptr_correspondences, this->config_.voxel_size, adjacent_voxels, points.size());

    stream.wait();
    nvtxRangePop();
  }

  /**
   * @brief Add points to the map
   * @param [in] points             Points to add
   * @param [in] map_density        Map density to use for adding points (either max points per
   * voxel or adaptive map density)
   * @param [in] adjacent_voxels    Number of adjacent voxels to search for
   * @param [in] num_neighbors      Number of neighbors to search for
   * @param [in] use_active_map     Flag to indicate if the active or inactive map should be used
   * @param [in] stream             Reference to the CUDA stream to use
   */
  __host__ void add_points(const std::vector<types::Point<types::Point_XYZ>>& points,
    const std::variant<int16_t, types::AdaptiveMapDensity>& map_density = TConfig::MAX_POINTS_PER_VOXEL,
    const int16_t adjacent_voxels = 1, const int16_t num_neighbors = TConfig::NUM_NEIGHBORS,
    const bool use_active_map = true, ::cuda::stream_ref stream = {}) override
  {
    // Copy points to device
    // Resize the vector to the size of the points
    this->points2insert_.resize(points.size());
    thrust::copy(points.begin(), points.end(), this->points2insert_.begin());
    // Call the device version of add_points
    this->add_points_device(this->points2insert_, map_density, adjacent_voxels, num_neighbors, use_active_map, stream);

    // Clear the member variable if its a global map
    if (!this->config_.frame_map) {
      this->points2insert_.clear();
      this->points2insert_.shrink_to_fit();  // Free memory
      stream.wait();
    }
  }

  /**
   * @brief Add points to the map (points on device)
   * @param [in] points             Points to add
   * @param [in] map_density        Map density to use for adding points (either max points per
   * voxel or adaptive map density)
   * @param [in] adjacent_voxels    Number of adjacent voxels to search for
   * @param [in] num_neighbors      Number of neighbors to search for
   * @param [in] use_active_map     Flag to indicate if the active or inactive map should be used
   * @param [in] stream             Reference to the CUDA stream to use
   * @note Callers using use_active_map = false are expected to hold the base class's
   * map_async_update_mutex during the call to ensure that no concurrent operations intervene.
   */
  __host__ void add_points_device(const thrust::device_vector<types::Point<types::Point_XYZ>>& device_points,
    const std::variant<int16_t, types::AdaptiveMapDensity>& map_density = TConfig::MAX_POINTS_PER_VOXEL,
    const int16_t adjacent_voxels = 1, const int16_t num_neighbors = TConfig::NUM_NEIGHBORS,
    const bool use_active_map = true, ::cuda::stream_ref stream = {}) override
  {
    // Get reference to the target map and the member variables
    // clang-format off
    StaticMapType & target_map = use_active_map ? this->get_active_map() : this->get_inactive_map();
    auto & target_keys = use_active_map ? this->get_active_keys() : this->get_inactive_keys();
    auto & target_pointers = use_active_map ? this->get_active_pointers() : this->get_inactive_pointers();  // NOLINT
    auto & target_structs = use_active_map ? this->get_active_structs() : this->get_inactive_structs();  // NOLINT
    auto & target_neighbors = use_active_map ? this->get_active_neighbors() : this->get_inactive_neighbors();  // NOLINT
    size_t & target_map_size = use_active_map ? this->get_active_map_size() : this->get_inactive_map_size();  // NOLINT
    unsigned int * target_counter = use_active_map ? this->counter_d_ : this->counter_d_async_;

    // Clear map if frame map is enabled or its the inactive map
    if (this->config_.frame_map || !use_active_map) {
      target_map.clear();
      target_map_size = 0;
    }
    nvtxRangePush("add_points");
    // Create copy to be able to modify the points
    thrust::device_vector<types::Point<types::Point_XYZ>> points_xyz = device_points;
    // Downsample points to reduce amount of memory needed
    voxel_downsample(
      points_xyz, this->points_downsampled_, this->config_.voxel_size, 50000, stream);

    // Insert voxels into the hashmap
    this->setup_voxel_grid(
      target_map, this->points_downsampled_, target_keys, target_pointers, target_structs, stream);

    // Check if the map size exceeds the maximum map size
    if (target_keys.size() >= types::MAX_MAP_SIZE) {
      throw std::runtime_error(
        "[cuda::VoxelHashMap]: Voxel grid size exceeds maximum map size. Please increase the "
        "maximum map size.");
    }

    // Insert points into hashmap
    nvtxRangePush("insert_points");
    target_neighbors.resize(target_keys.size() * TConfig::MAX_POINTS_PER_VOXEL);
    const types::Point<types::Point_XYZ> * raw_ptr_points =
      thrust::raw_pointer_cast(device_points.data());
    types::Neighbors<TConfig> * raw_ptr_neighbors =
      thrust::raw_pointer_cast(target_neighbors.data());
    // Index to keep track of the number of points inserted
    // -> Needed to copy the pointers to the points in the map
    unsigned int h_counter = 0;
    cudaMemset(target_counter, 0, sizeof(unsigned int));

    // Set new grid size for insert values kernel
    auto grid_size = std::min(
      4 * this->num_multiprocessors_, (device_points.size() + BLOCK_SIZE - 1) / BLOCK_SIZE);

    // Launch the kernel based on the map density type
    if (std::holds_alternative<types::AdaptiveMapDensity>(map_density)) {
      const types::AdaptiveMapDensity & density = std::get<types::AdaptiveMapDensity>(map_density);
      insert_values_kernel<<<grid_size, BLOCK_SIZE, 0, stream.get()>>>(
        target_map.ref(cuco::find), raw_ptr_points, raw_ptr_neighbors, target_counter,
        device_points.size(), this->config_.voxel_size, this->get_resolution(), density.max_points,
        density.min_points, density.range, density.max_points_scale, density.origin.x(),
        density.origin.y(), density.origin.z());
    } else {
      insert_values_kernel<<<grid_size, BLOCK_SIZE, 0, stream.get()>>>(
        target_map.ref(cuco::find), raw_ptr_points, raw_ptr_neighbors, target_counter,
        device_points.size(), this->config_.voxel_size, this->get_resolution(),
        std::get<int16_t>(map_density));
    }

    stream.wait();
    cudaMemcpy(&h_counter, target_counter, sizeof(unsigned int), cudaMemcpyDeviceToHost);
    // Resize the map correspondences vector to the number of points inserted
    target_neighbors.resize(h_counter);
    nvtxRangePop();

    // Set normals and covariances to points (if enabled)
    // -> Needs to be done after the points are inserted into the map (otherwise correspondence
    // search not possible)
    if constexpr (types::HASNORMALCOV<TConfig>) {
      nvtxRangePush("set_normal_covariance");
      // Start timer
      auto start = std::chrono::high_resolution_clock::now();

      if (!target_neighbors.empty()) {
        const int16_t num_neighbors_clamped = std::min(num_neighbors, TConfig::NUM_NEIGHBORS);
        // Search for the closest neighbors
        this->search_closest_neighbors(
          target_neighbors, adjacent_voxels, num_neighbors_clamped, use_active_map, stream);

        // Set normals and covariances
        this->set_normal_covariance(
          target_neighbors, num_neighbors_clamped, this->num_multiprocessors_, stream);
      }

      // Compute elapsed time
      // clang-format off
      double normal_cov_time = std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::high_resolution_clock::now() - start).count() * 1.0e-3;  // NOLINT
      // clang-format on
      // Update debug information
      if (use_active_map) this->debug_.normal_cov_time = normal_cov_time;
      nvtxRangePop();
    }
    stream.wait();

    // Update debug information
    if (!this->config_.frame_map) {
      // Set map size
      target_map_size = target_map.size();
      if (!use_active_map) return;
      this->check_load_factor(this->get_active_map_size());
      this->debug_.num_voxel = this->get_active_map_size();
      this->debug_.num_points = this->num_points(stream);
    }
    nvtxRangePop();
  }

  /**
   * @brief Update the map with new points
   * @param [in] points             Points to update
   * @param [in] pose               Pose the points are in
   * @param [in] map_density        Map density to use for adding points (either max points per
   * voxel or adaptive map density)
   * @param [in] adjacent_voxels    Number of adjacent voxels to search for
   * @param [in] num_neighbors      Number of neighbors to search for
   * @param [in] stream             Reference to the CUDA stream to use
   */
  __host__ void update_points(const thrust::device_vector<types::Point<TConfig>>& points, const Sophus::SE3f& pose,
    const std::variant<int16_t, types::AdaptiveMapDensity>& map_density = TConfig::MAX_POINTS_PER_VOXEL,
    const int16_t adjacent_voxels = 1, const int16_t num_neighbors = TConfig::NUM_NEIGHBORS,
    ::cuda::stream_ref stream = {})
  {
    // Start timer
    auto start = std::chrono::high_resolution_clock::now();

    // Transform points to map frame and map point type
    thrust::device_vector<types::Point<types::Point_XYZ>> points_transformed(points.size());
    thrust::transform(thrust::cuda::par.on(stream.get()), points.begin(), points.end(), points_transformed.begin(),
      [] __device__(const types::Point<TConfig>& src) { return utils::convert_point<TConfig, types::Point_XYZ>(src); });
    utils::transform_points(pose, points_transformed, stream);
    const Eigen::Vector3f& origin = pose.translation();

    // Insert points into the map
    this->insert_points(points_transformed, map_density, stream);

    // Update normals and covariances if enabled
    // NOTE: needs to be done before removal, as otherwise the pointers
    // become invalid
    if constexpr (types::HASNORMALCOV<TConfig>) {
      nvtxRangePush("set_normal_covariance");

      // Start timer
      auto normal_cov_start = std::chrono::high_resolution_clock::now();

      if (!this->get_active_neighbors().empty()) {
        const int16_t num_neighbors_clamped = std::min(num_neighbors, TConfig::NUM_NEIGHBORS);
        // Search for the closest neighbors
        this->search_closest_neighbors(
          this->get_active_neighbors(), adjacent_voxels, num_neighbors_clamped, true, stream);

        // Set normals and covariances
        this->set_normal_covariance(
          this->get_active_neighbors(), num_neighbors_clamped, this->num_multiprocessors_, stream);
      }

      // Compute elapsed time
      // clang-format off
      double normal_cov_time = std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::high_resolution_clock::now() - normal_cov_start).count() * 1.0e-3;  // NOLINT
      // clang-format on
      // Update debug information
      this->debug_.normal_cov_time = normal_cov_time;
      nvtxRangePop();
    }

    // Remove far points from the map
    this->remove_far_points(origin, stream);

    // Swap map instances for future iterations
    this->swap_map(stream);

    // Check the load factor of the hashmap
    this->check_load_factor(this->get_active_map_size());

    // Compute elapsed time
    // clang-format off
    double time = std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::high_resolution_clock::now() - start).count() * 1.0e-3;  // NOLINT
    // clang-format on
    // Update debug information
    this->debug_.update_time = time;
  }

  /**
   * @brief Get the map density
   * @param [in] range_close          Range to consider as close
   * @param [in] range_far            Range to consider as far
   * @return                          AdaptiveMap density
   */
  __host__ types::AdaptiveMapDensity get_density(double range, ::cuda::stream_ref stream = {})
  {
    nvtxRangePush("density");
    unsigned int h_counter_voxels = 0;
    unsigned int h_counter_points = 0;

    cudaMemset(this->counter_d_, 0, sizeof(unsigned int));
    cudaMemset(this->counter2_d_, 0, sizeof(unsigned int));

    unsigned int* counter_voxels_ptr = this->counter_d_;
    unsigned int* counter_points_ptr = this->counter2_d_;
    double range2 = range * range;

    this->get_active_map().for_each(
      [counter_voxels_ptr, counter_points_ptr, range2] __device__(auto const& slot) {
        if ((slot.second->data[0].pos).squaredNorm() < range2) {
          atomicAdd(counter_voxels_ptr, 1);
          atomicAdd(counter_points_ptr, slot.second->counter);
        }
      },
      stream.get());

    stream.wait();
    // clang-format off
    cudaMemcpy(&h_counter_voxels, this->counter_d_, sizeof(unsigned int), cudaMemcpyDeviceToHost);  // NOLINT
    cudaMemcpy(&h_counter_points, this->counter2_d_, sizeof(unsigned int), cudaMemcpyDeviceToHost);  // NOLINT
    nvtxRangePop();
    types::AdaptiveMapDensity density{};
    density.range = range;
    density.max_points = (h_counter_voxels > 0) ? static_cast<std::int16_t>(h_counter_points / h_counter_voxels) : 0;  // NOLINT
    density.min_points = TConfig::NUM_NEIGHBORS;
    density.max_points_scale = -0.03f;
    density.valid = density.max_points > 0;
    // clang-format on
    return density;
  }

  /**
   * @brief Get the map as a point cloud
   * @return                        Point cloud of the map
   */
  __host__ thrust::device_vector<types::Point<TConfig>> get_cloud(::cuda::stream_ref stream = {}) const override
  {
    nvtxRangePush("get_cloud");
    thrust::device_vector<types::Point<TConfig>> device_points(this->num_points(stream));
    types::Point<TConfig>* device_points_ptr = thrust::raw_pointer_cast(device_points.data());

    // Setup counter - we dont need to copy it afterwards though
    cudaMemset(this->counter_d_, 0, sizeof(unsigned int));
    unsigned int* counter_ptr = this->counter_d_;
    // Collect keys to erase
    this->get_active_map().for_each(
      [device_points_ptr, counter_ptr] __device__(auto const& slot) {
        // Copy points to device points vector
        for (int16_t i = 0; i < slot.second->counter; ++i) {
          unsigned int index = atomicAdd(counter_ptr, 1);
          device_points_ptr[index] = slot.second->data[i];
        }
      },
      stream.get());
    stream.wait();
    nvtxRangePop();
    return device_points;
  }

  /**
   * @brief Get the vector holding the neighbors of the points in the map
   * @return                        Neighbors of the points in the map
   * @note this is required for testing of the muliple nearest neighbor search
   */
  __host__ thrust::device_vector<types::Neighbors<TConfig>> get_neighbors() const override
  {
    // Return the neighbors vector
    return this->get_active_neighbors();
  }

  /**
   * @brief Switch the active map
   */
  void switch_active_map() override
  {
    // Switch active map
    this->map_a_active_.store(!this->map_a_active_.load());
    // Update debug information as not set in async load
    this->check_load_factor(this->get_active_map_size());
    this->debug_.num_voxel = this->get_active_map_size();
    // This is done on the default stream
    this->debug_.num_points = this->num_points();
  }

protected:
  // Inherit constructor from MapHandler for param manager and logger
  VoxelHashMap(tam::pmg::ParamReferenceManager* pmg, tam::tsl::ReferenceLogger* logger)
      : MapHandler<TConfig>(pmg, logger)
  {
    this->setup_cuda_device();
  }

  // Inherit constructor from MapHandler for config and debug object
  VoxelHashMap(const types::MapConfig& config, const types::MapDebug& debug) : MapHandler<TConfig>(config, debug)
  {
    this->setup_cuda_device();
  }

  /**
   * @brief Search for the closest neighbors of a point
   * @param [in] correspondences      vector to return the correspondences in
   * @param [in] adjacent_voxels          Number of adjacent voxels to search in
   * @param [in] num_neighbors            Number of neighbors to search for
   * @param [in] use_active_map     Flag to indicate if the active or inactive map should be used
   * @param [in] stream               Reference to the CUDA stream to use
   */
  __host__ void search_closest_neighbors(thrust::device_vector<types::Neighbors<TConfig>>& neighbors,
    const int16_t adjacent_voxels, const int16_t num_neighbors, const bool use_active_map = true,
    ::cuda::stream_ref stream = {}) override
  {
    nvtxRangePush("search_closest_neighbors");

    // Get reference to the target map
    const StaticMapType& target_map = use_active_map ? this->get_active_map() : this->get_inactive_map();

    auto grid_size = std::min(4 * this->num_multiprocessors_, (neighbors.size() + BLOCK_SIZE - 1) / BLOCK_SIZE);

    types::Neighbors<TConfig>* raw_ptr_neighbors = thrust::raw_pointer_cast(neighbors.data());

    search_closest_neighbors_kernel<<<grid_size, BLOCK_SIZE, 0, stream.get()>>>(target_map.ref(cuco::find),
      raw_ptr_neighbors, this->config_.voxel_size, adjacent_voxels, num_neighbors, neighbors.size());

    stream.wait();
    nvtxRangePop();
  }

public:
  /**
   * @brief Setup the voxel grid and insert points into the map
   * @param [in] map                 Reference to the map to insert points into
   * @param [in] points              Points to insert into the map
   * @param [out] voxel_keys         Vector to hold the voxel keys
   * @param [out] pointer_vector     Vector to hold pointers to the fixed sized points
   * @param [out] struct_vector      Vector to hold the fixed sized points
   * @param [in] stream              Reference to the CUDA stream to use
   */
  __host__ void setup_voxel_grid(StaticMapType& map,
    const thrust::device_vector<types::Point<types::Point_XYZ>>& points,
    thrust::device_vector<custom_voxel_type>& voxel_keys,
    thrust::device_vector<fixed_sized_points_array<TConfig>*>& pointer_vector,
    thrust::device_vector<fixed_sized_points_array<TConfig>>& struct_vector, ::cuda::stream_ref stream = {})
  {
    nvtxRangePush("setup_voxel_grid");
    // Capture voxel keys
    if (struct_vector.size() < points.size()) {
      struct_vector.resize(points.size() * 2);
      pointer_vector.resize(points.size() * 2);
    }
    voxel_keys.resize(points.size());
    double voxel_size = this->config_.voxel_size;
    thrust::transform(thrust::cuda::par.on(stream.get()), points.begin(), points.end(), voxel_keys.begin(),
      [voxel_size] __device__(const auto& point) { return PointToVoxel<types::Point_XYZ>(voxel_size)(point); });
    // Create struct and pointer vectors
    auto end_it_pointer_vector = pointer_vector.begin() + voxel_keys.size();
    auto end_it_struct_vector = struct_vector.begin() + voxel_keys.size();

    thrust::for_each(thrust::cuda::par.on(stream.get()),
      thrust::make_zip_iterator(thrust::make_tuple(pointer_vector.begin(), struct_vector.begin())),
      thrust::make_zip_iterator(thrust::make_tuple(end_it_pointer_vector, end_it_struct_vector)),
      [] __device__(auto tuple) {
        auto& voxel = thrust::get<1>(tuple);
        thrust::get<0>(tuple) = &voxel;
        voxel.counter = 0;
      });

    // Create zip iterator to insert the points into the hashmap
    auto zipped = thrust::make_zip_iterator(thrust::make_tuple(voxel_keys.begin(), pointer_vector.begin()));
    map.insert(zipped, zipped + voxel_keys.size());
    nvtxRangePop();
  }

  /**
   * @brief Insert points into existing map
   * @param [in] device_points       Points to insert
   * @param [in] map_density         Map density to use for adding points (either max points per
   * voxel or adaptive map density)
   * @param [in] stream              Reference to the CUDA stream to use
   */
  void __host__ insert_points(const thrust::device_vector<types::Point<types::Point_XYZ>>& device_points,
    const std::variant<int16_t, types::AdaptiveMapDensity>& map_density = TConfig::MAX_POINTS_PER_VOXEL,
    ::cuda::stream_ref stream = {})
  {
    nvtxRangePush("insert_points");
    // No downsampling executed as points already downsampled (although not same voxel size)
    this->setup_voxel_grid(this->get_active_map(), device_points, this->voxel_keys_inter_, this->pointer_vector_inter_,
      this->struct_vector_inter_, stream);

    // Insert values into hashmap
    nvtxRangePush("insert_values");
    this->get_active_neighbors().resize(voxel_keys_inter_.size() * TConfig::MAX_POINTS_PER_VOXEL);
    const types::Point<types::Point_XYZ>* raw_ptr_points = thrust::raw_pointer_cast(device_points.data());
    types::Neighbors<TConfig>* raw_ptr_neighbors = thrust::raw_pointer_cast(this->get_active_neighbors().data());
    // Index to keep track of the number of points inserted
    // -> Needed to copy the pointers to the points in the map
    unsigned int h_counter = 0;
    cudaMemset(this->counter_d_, 0, sizeof(unsigned int));

    // Set new grid size for insert values kernel
    auto grid_size = std::min(4 * this->num_multiprocessors_, (device_points.size() + BLOCK_SIZE - 1) / BLOCK_SIZE);

    // Launch the kernel based on the map density type
    if (std::holds_alternative<types::AdaptiveMapDensity>(map_density)) {
      const types::AdaptiveMapDensity& density = std::get<types::AdaptiveMapDensity>(map_density);
      insert_values_kernel<<<grid_size, BLOCK_SIZE, 0, stream.get()>>>(this->get_active_map().ref(cuco::find),
        raw_ptr_points, raw_ptr_neighbors, this->counter_d_, device_points.size(), this->config_.voxel_size,
        this->get_resolution(), density.max_points, density.min_points, density.range, density.max_points_scale,
        density.origin.x(), density.origin.y(), density.origin.z());
    } else {
      insert_values_kernel<<<grid_size, BLOCK_SIZE, 0, stream.get()>>>(this->get_active_map().ref(cuco::find),
        raw_ptr_points, raw_ptr_neighbors, this->counter_d_, device_points.size(), this->config_.voxel_size,
        this->get_resolution(), std::get<int16_t>(map_density));
    }
    stream.wait();
    cudaMemcpy(&h_counter, this->counter_d_, sizeof(unsigned int), cudaMemcpyDeviceToHost);
    // Resize the map correspondences vector to the number of points inserted
    this->get_active_neighbors().resize(h_counter);
    nvtxRangePop();

    // Update debug information
    this->get_active_map_size() = this->get_active_map().size();
    this->debug_.points_added = h_counter;
    this->debug_.voxel_added = this->get_active_map_size() - this->debug_.num_voxel;
    this->debug_.num_voxel = this->get_active_map_size();
    nvtxRangePop();
  }

  /**
   * @brief Remove points that are too far from the origin
   * @param [in] origin              current origin
   * @note The device lambda requires the function to be public within the class
   */
  void __host__ remove_far_points(const Eigen::Vector3f& origin, ::cuda::stream_ref stream = {})
  {
    nvtxRangePush("remove_far_points");
    const double max_distance2 = this->config_.max_distance * this->config_.max_distance;
    this->keys2erase_.resize(this->get_active_map_size());
    custom_voxel_type* keys2erase_ptr = thrust::raw_pointer_cast(this->keys2erase_.data());

    // Setup counter for keys to erase
    nvtxRangePush("collect_keys");
    unsigned int h_counter = 0;
    cudaMemset(this->counter_d_, 0, sizeof(unsigned int));
    unsigned int* counter_ptr = this->counter_d_;
    // Collect keys to erase
    this->get_active_map().for_each(
      [origin, max_distance2, keys2erase_ptr, counter_ptr] __device__(auto const& slot) {
        // Check if first point in the voxel is too far from the origin
        if ((slot.second->data[0].pos - origin).squaredNorm() > max_distance2) {
          unsigned int index = atomicAdd(counter_ptr, 1);
          keys2erase_ptr[index] = slot.first;  // Store the key to erase
        }
      },
      stream.get());
    stream.wait();
    // Copy counter to host
    cudaMemcpy(&h_counter, this->counter_d_, sizeof(unsigned int), cudaMemcpyDeviceToHost);
    this->keys2erase_.resize(h_counter);
    nvtxRangePop();

    nvtxRangePush("erase_keys");
    // Erase the keys from the map
    this->get_active_map().erase(this->keys2erase_.begin(), this->keys2erase_.end(), stream.get());
    nvtxRangePop();

    // Update debug information
    const auto num_points = this->num_points(stream);
    this->get_active_map_size() = this->get_active_map_size() - h_counter;
    this->debug_.voxel_removed = h_counter;
    this->debug_.num_voxel = this->get_active_map_size();
    this->debug_.points_removed = num_points - this->debug_.num_points - this->debug_.points_added;
    this->debug_.num_points = num_points;
    nvtxRangePop();
  }

  /**
   * @brief Move points to inactive map instance and change map states to allow iterative updating
   */
  void __host__ swap_map(::cuda::stream_ref stream = {})
  {
    // Get the mutex as we are about to use the inactive map
    // which is also used for asynchronous map updates
    std::lock_guard<std::mutex> lock(this->mutex_);
    // If another thread wrote an update to the map, use it
    if (this->switch_pending_.load()) {
      this->switch_active_map();
      this->switch_pending_.store(false);
      return;
    }
    nvtxRangePush("swap_map");
    // only resize if the initial memory pool is not sufficient
    if (this->get_inactive_keys().size() < this->get_active_map_size()) {
      this->get_inactive_keys().resize(this->get_active_map_size() * 2);
      this->get_inactive_pointers().resize(this->get_active_map_size() * 2);
      this->get_inactive_structs().resize(this->get_active_map_size() * 2);
      this->pointer_vector_swap_map_.resize(this->get_active_map_size() * 2);
    }
    // Retrieve the keys of the map
    nvtxRangePush("retrieve_keys");
    [[maybe_unused]] auto [keys_end, values_end] = this->get_active_map().retrieve_all(
      this->get_inactive_keys().begin(), pointer_vector_swap_map_.begin(), stream.get());
    nvtxRangePop();

    // Setup the second map
    nvtxRangePush("setup_new_map");
    // clang-format off
    auto end_it_pointer_vector = this->get_inactive_pointers().begin() + this->get_active_map_size();  // NOLINT
    auto end_it_struct_vector = this->get_inactive_structs().begin() + this->get_active_map_size();  // NOLINT
    // clang-format on
    thrust::for_each(thrust::cuda::par.on(stream.get()),
      thrust::make_zip_iterator(
        thrust::make_tuple(this->get_inactive_pointers().begin(), this->get_inactive_structs().begin())),
      thrust::make_zip_iterator(thrust::make_tuple(end_it_pointer_vector, end_it_struct_vector)),
      [] __device__(auto tuple) { thrust::get<0>(tuple) = &thrust::get<1>(tuple); });
    stream.wait();

    // Create zip iterator to insert the points into the hashmap
    auto zipped = thrust::make_zip_iterator(
      thrust::make_tuple(this->get_inactive_keys().begin(), this->get_inactive_pointers().begin()));
    this->get_inactive_map().insert(zipped, zipped + this->get_active_map_size());
    nvtxRangePop();

    // Copy the values to the second map
    nvtxRangePush("copy_values");
    const custom_voxel_type* keys_ptr = thrust::raw_pointer_cast(this->get_inactive_keys().data());
    size_t num_voxels = this->get_active_map_size();
    int16_t grid_size = (num_voxels + BLOCK_SIZE - 1) / BLOCK_SIZE;
    copy_values_kernel<TConfig><<<grid_size, BLOCK_SIZE, 0, stream.get()>>>(
      this->get_active_map().ref(cuco::find), this->get_inactive_map().ref(cuco::find), keys_ptr, num_voxels);
    stream.wait();

    nvtxRangePop();
    this->get_active_map().clear();
    this->get_inactive_map_size() = this->get_active_map_size();
    this->switch_active_map();
    nvtxRangePop();
  }

private:
  /**
   * @brief Get active and inactive map references
   * @note Cannot be in base class as reference type differs
   */
  // clang-format off
  __host__ StaticMapType & get_active_map() { return this->map_a_active_.load() ? map_a_ : map_b_; }  // NOLINT
  __host__ const StaticMapType & get_active_map() const { return this->map_a_active_.load() ? map_a_ : map_b_; }  // NOLINT
  __host__ size_t & get_active_map_size() { return this->map_a_active_.load() ? map_a_size_ : map_b_size_; }  // NOLINT
  __host__ thrust::device_vector<custom_voxel_type> & get_active_keys() { return this->map_a_active_.load() ? voxel_keys_a_ : voxel_keys_b_; }  // NOLINT
  __host__ thrust::device_vector<fixed_sized_points_array<TConfig> *> & get_active_pointers() { return this->map_a_active_.load() ? pointer_vector_a_ : pointer_vector_b_; }  // NOLINT
  __host__ thrust::device_vector<fixed_sized_points_array<TConfig>> & get_active_structs() { return this->map_a_active_.load() ? struct_vector_a_ : struct_vector_b_; }  // NOLINT
  __host__ thrust::device_vector<types::Neighbors<TConfig>> & get_active_neighbors() { return this->map_a_active_.load() ? neighbors_a_ : neighbors_b_; }  // NOLINT
  __host__ const thrust::device_vector<types::Neighbors<TConfig>> & get_active_neighbors() const { return this->map_a_active_.load() ? neighbors_a_ : neighbors_b_; }  // NOLINT
  __host__ StaticMapType & get_inactive_map() { return this->map_a_active_.load() ? map_b_ : map_a_; }  // NOLINT
  __host__ const StaticMapType & get_inactive_map() const { return this->map_a_active_.load() ? map_b_ : map_a_; }  // NOLINT
  __host__ size_t & get_inactive_map_size() { return this->map_a_active_.load() ? map_b_size_ : map_a_size_; }  // NOLINT
  __host__ thrust::device_vector<custom_voxel_type> & get_inactive_keys() { return this->map_a_active_.load() ? voxel_keys_b_ : voxel_keys_a_; }  // NOLINT
  __host__ thrust::device_vector<fixed_sized_points_array<TConfig> *> & get_inactive_pointers() { return this->map_a_active_.load() ? pointer_vector_b_ : pointer_vector_a_; }  // NOLINT
  __host__ thrust::device_vector<fixed_sized_points_array<TConfig>> & get_inactive_structs() { return this->map_a_active_.load() ? struct_vector_b_ : struct_vector_a_; }  // NOLINT
  __host__ thrust::device_vector<types::Neighbors<TConfig>> & get_inactive_neighbors() { return this->map_a_active_.load() ? neighbors_b_ : neighbors_a_; }  // NOLINT
  __host__ const thrust::device_vector<types::Neighbors<TConfig>> & get_inactive_neighbors() const { return this->map_a_active_.load() ? neighbors_b_ : neighbors_a_; }  // NOLINT

  // clang-format on

  /**
   * @brief Setup CUDA device
   * @param [in] device_id               Device ID to use
   */
  void __host__ setup_cuda_device(int device_id = 0)
  {
    // Set the device
    this->num_multiprocessors_ = utils::set_device(device_id).multiProcessorCount;
    // Allocate the counters on the device
    cudaMalloc(&counter_d_, sizeof(unsigned int));
    cudaMalloc(&counter2_d_, sizeof(unsigned int));
    cudaMalloc(&counter_d_async_, sizeof(unsigned int));
    cudaMemset(counter_d_, 0, sizeof(unsigned int));
    cudaMemset(counter2_d_, 0, sizeof(unsigned int));
    cudaMemset(counter_d_async_, 0, sizeof(unsigned int));
    // Synchronize to ensure the context is initialized
    cudaDeviceSynchronize();
  }

  /**
   * @brief Allocate map switching vectors
   * @param [in] update_map          Whether the map is to be updated
   * @param [in] size                Size to allocate
   */
  void allocate_memory([[maybe_unused]] const bool update_map, [[maybe_unused]] const size_t size) override
  {
    // Preallocate vectors on the GPU
    // Only allocate space if the map is a frame map
    utils::allocate_vector(points2insert_, size);
    utils::allocate_vector(points_downsampled_, size);
    utils::allocate_vector(voxel_keys_a_, size);
    utils::allocate_vector(pointer_vector_a_, size);
    utils::allocate_vector(struct_vector_a_, size);
    utils::allocate_vector(neighbors_a_, size * TConfig::MAX_POINTS_PER_VOXEL);
    // Allocate vectors for map b if the map is to be updated
    if (update_map) {
      utils::allocate_vector(voxel_keys_b_, size);
      utils::allocate_vector(pointer_vector_b_, size);
      utils::allocate_vector(struct_vector_b_, size);
      utils::allocate_vector(neighbors_b_, size * TConfig::MAX_POINTS_PER_VOXEL);
      utils::allocate_vector(pointer_vector_swap_map_, size);
      utils::allocate_vector(voxel_keys_inter_, size);
      utils::allocate_vector(pointer_vector_inter_, size);
      utils::allocate_vector(struct_vector_inter_, size);
      utils::allocate_vector(keys2erase_, size);
    }
  }

  /**
   * @brief Check load factor of hashmap
   */
  __host__ void check_load_factor(size_t map_size)
  {
    // clang-format off
    this->debug_.load_factor = static_cast<float>(map_size) / static_cast<float>(types::MAX_MAP_SIZE); // NOLINT
    if (this->debug_.load_factor > types::MAX_LOAD_FACTOR) {
      std::cerr << "\033[1;33m[cuda::VoxelHashMap]: High load factor of hashmap: " << this->debug_.load_factor << "\033[0m" << std::endl;  // NOLINT
    }
    // clang-format on
  }

private:
  // Number of multiprocessors on the device
  size_t num_multiprocessors_{0};
  // Counter variable
  mutable unsigned int* counter_d_;
  mutable unsigned int* counter2_d_;
  mutable unsigned int* counter_d_async_;
  // CUDA Hashmap
  custom_voxel_type const empty_key_sentinel = custom_voxel_type{-1};
  custom_voxel_type const erased_key_sentinel = custom_voxel_type{-2};
  fixed_sized_points_array<TConfig>* empty_value_sentinel = nullptr;

  StaticMapType map_a_{cuco::extent<std::size_t, types::MAX_MAP_SIZE>{}, cuco::empty_key{empty_key_sentinel},
    cuco::empty_value{empty_value_sentinel}, cuco::erased_key{erased_key_sentinel}, custom_key_equal{},
    cuco::linear_probing<1, custom_hash>{}};

  StaticMapType map_b_{cuco::extent<std::size_t, types::MAX_MAP_SIZE>{}, cuco::empty_key{empty_key_sentinel},
    cuco::empty_value{empty_value_sentinel}, cuco::erased_key{erased_key_sentinel}, custom_key_equal{},
    cuco::linear_probing<1, custom_hash>{}};

  // store the fixed_sized_points_arrays containing the points in the map as member
  // to ensure the variables never fall out of scope
  // map a instance
  size_t map_a_size_{0};
  thrust::device_vector<custom_voxel_type> voxel_keys_a_;
  thrust::device_vector<fixed_sized_points_array<TConfig>*> pointer_vector_a_;
  thrust::device_vector<fixed_sized_points_array<TConfig>> struct_vector_a_;
  thrust::device_vector<types::Neighbors<TConfig>> neighbors_a_;
  // map b instance
  size_t map_b_size_{0};
  thrust::device_vector<custom_voxel_type> voxel_keys_b_;
  thrust::device_vector<fixed_sized_points_array<TConfig>*> pointer_vector_b_;
  thrust::device_vector<fixed_sized_points_array<TConfig>> struct_vector_b_;
  thrust::device_vector<types::Neighbors<TConfig>> neighbors_b_;
  // Intermediate map inserting
  thrust::device_vector<fixed_sized_points_array<TConfig>*> pointer_vector_swap_map_;
  thrust::device_vector<custom_voxel_type> voxel_keys_inter_;
  thrust::device_vector<fixed_sized_points_array<TConfig>*> pointer_vector_inter_;
  thrust::device_vector<fixed_sized_points_array<TConfig>> struct_vector_inter_;

  // Member variables for point insertion for frame map and normal estimation
  thrust::device_vector<types::Point<types::Point_XYZ>> points2insert_;
  thrust::device_vector<types::Point<types::Point_XYZ>> points_downsampled_;
  thrust::device_vector<custom_voxel_type> keys2erase_;
};
}  // namespace tam::core::state::cuda
