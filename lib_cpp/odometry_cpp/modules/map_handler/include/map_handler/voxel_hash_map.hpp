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
// Implementation modified from
// https://github.com/PRBonn/kiss-icp/blob/main/cpp/kiss_icp/core/VoxelHashMap.hpp
#pragma once

#include <tsl/robin_map.h>

#include <algorithm>
#include <chrono>
#include <functional>
#include <limits>
#include <memory>
#include <queue>
#include <sophus/se3.hpp>
#include <string>
#include <tuple>
#include <unordered_set>
#include <utility>
#include <vector>

#include "map_handler/map_handler_base.hpp"
#include "voxel_tools/voxel_tools.hpp"

// Required as tsl namespace collides with tam::tsl namespace
using tsl::robin_map;

namespace tam::core::state {
template <typename TConfig>
class VoxelHashMap : public MapHandler<TConfig>
{
public:
  // Define MapType
  using MapType = robin_map<Voxel, std::vector<types::Point<TConfig>>>;

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
  void clear() override
  {
    map_a_.clear();
    map_b_.clear();
    this->debug_.num_voxel = 0;
    this->debug_.num_points = 0;
  }

  /**
   * @brief Check if map is empty
   */
  bool empty() const override { return this->get_active_map().empty(); }

  /**
   * @brief Get the amount of points in the map
   * @return                        Number of points in the map
   */
  std::size_t num_points() const override
  {
    return std::accumulate(this->get_active_map().cbegin(), this->get_active_map().cend(), 0,
      [](const size_t& sum, const auto& map_element) { return sum + map_element.second.size(); });
  }

  /**
   * @brief search for closest neighbor of a point
   * @param [in] point              Point to search for
   * @param [in] adjacent_voxels    Number of adjacent voxels to search in each direction
   * @return                        Closest point and distance
   */
  types::Correspondence<TConfig> search_closest_neighbor(
    const types::Point<TConfig>& point, const int16_t adjacent_voxels = 1) const override
  {
    // Convert the point to voxel coordinates
    const auto& voxel = point_to_voxel<TConfig>(point, this->config_.voxel_size);
    // Get nearby voxels on the map
    const auto& neighbor_voxels = get_adjacent_voxels(voxel, adjacent_voxels);
    // Iterate over the neighbor voxels
    std::vector<types::Point<TConfig>> neighbors{};
    int16_t num_neighbors = (2 * adjacent_voxels + 1) * (2 * adjacent_voxels + 1) * (2 * adjacent_voxels + 1);
    neighbors.reserve(num_neighbors * TConfig::MAX_POINTS_PER_VOXEL);
    std::for_each(neighbor_voxels.cbegin(), neighbor_voxels.cend(), [&](const auto& neighbor) {
      auto search = this->get_active_map().find(neighbor);
      if (search != this->get_active_map().end()) {
        const auto& map_points = search.value();
        if (!map_points.empty()) {
          for (const auto& map_point : map_points) {
            neighbors.emplace_back(map_point);
          }
        }
      }
    });
    // Find the closest neighbor
    types::Correspondence<TConfig> correspondence{};
    correspondence.frame = point;
    correspondence.distance = std::numeric_limits<double>::max();
    if (!neighbors.empty()) {
      std::for_each(neighbors.cbegin(), neighbors.cend(), [&](const auto& neighbor) {
        float distance = (neighbor.pos - point.pos).squaredNorm();
        if (distance < correspondence.distance) {
          correspondence.map = neighbor;
          correspondence.distance = distance;
        }
      });
    }
    return correspondence;
  }

  /**
   * @brief Add points to the map
   * @param [in] points             Points to add
   * @param [in] map_density        Map density to use for adding points (either max points per
   * voxel or adaptive map density)
   * @param [in] adjacent_voxels    Number of adjacent voxels to search for
   * @param [in] num_neighbors      Number of neighbors to search for
   * @param [in] use_active_map     Flag to indicate if the active or inactive map should be used
   * @note Callers using use_active_map = false are expected to hold the base class's
   * map_async_update_mutex during the call to ensure that no concurrent operations intervene.
   */
  void add_points(const std::vector<types::Point<types::Point_XYZ>>& points,
    const std::variant<int16_t, types::AdaptiveMapDensity>& map_density = TConfig::MAX_POINTS_PER_VOXEL,
    const int16_t adjacent_voxels = 1, const int16_t num_neighbors = TConfig::NUM_NEIGHBORS,
    const bool use_active_map = true) override
  {
    // Get reference to the target map and the neighbors vector
    MapType& target_map = use_active_map ? this->get_active_map() : this->get_inactive_map();
    tbb::concurrent_vector<types::Neighbors<TConfig>>& target_neighbors =
      use_active_map ? this->get_active_neighbors() : this->get_inactive_neighbors();

    // Clear map if frame map is enabled or its the inactive map
    if (this->config_.frame_map || !use_active_map) {
      target_map.clear();
    }

    // Reserve space for occupied voxel caching and neighbors
    target_neighbors.clear();
    target_neighbors.reserve(points.size());
    const double map_resolution = this->get_resolution();
    unsigned int counter = 0;
    std::for_each(points.cbegin(), points.cend(), [&](const auto& point) {
      const auto voxel = point_to_voxel<types::Point_XYZ>(point, this->config_.voxel_size);
      auto search = target_map.find(voxel);
      if (search != target_map.end()) {
        std::vector<types::Point<TConfig>>& voxel_points = search.value();
        // Compute max number of points within this voxel
        int16_t max_points_per_voxel = TConfig::MAX_POINTS_PER_VOXEL;
        if (std::holds_alternative<types::AdaptiveMapDensity>(map_density)) {
          // clang-format off
          const auto & adaptive_map_density = std::get<types::AdaptiveMapDensity>(map_density);
          const float distance2origin = (point.pos - adaptive_map_density.origin).norm();
          if (distance2origin < adaptive_map_density.range) {
            max_points_per_voxel = std::max(adaptive_map_density.min_points, adaptive_map_density.max_points);  // NOLINT
          } else {
            max_points_per_voxel = std::max(
              adaptive_map_density.min_points,
              static_cast<int16_t>(adaptive_map_density.max_points + (distance2origin - adaptive_map_density.range) *adaptive_map_density.max_points_scale));  // NOLINT
          }
          // clang-format on
        } else {
          max_points_per_voxel = std::get<int16_t>(map_density);
        }
        if (voxel_points.size() >= static_cast<size_t>(max_points_per_voxel) ||
          std::any_of(voxel_points.cbegin(), voxel_points.cend(),
            [&](const auto& voxel_point) { return (voxel_point.pos - point.pos).norm() < map_resolution; })) {
          // Voxel is already full
          return;
        }
        voxel_points.emplace_back(utils::convert_point<types::Point_XYZ, TConfig>(point));
        counter++;
        // Store pointer to point in tbb_neighbors_
        types::Neighbors<TConfig> neighbors{};
        neighbors.point = &voxel_points.back();
        target_neighbors.emplace_back(neighbors);
      } else {
        std::vector<types::Point<TConfig>> voxel_points;
        // Always reserve for maximum amount of points (compile time param)
        voxel_points.reserve(TConfig::MAX_POINTS_PER_VOXEL);
        voxel_points.emplace_back(utils::convert_point<types::Point_XYZ, TConfig>(point));
        target_map.insert({voxel, std::move(voxel_points)});
        counter++;
        // Store pointer to point in tbb_neighbors_
        types::Neighbors<TConfig> neighbors{};
        neighbors.point = &target_map[voxel].back();
        target_neighbors.emplace_back(neighbors);
      }
    });
    // Set normals and covariances to points (only if enabled)
    if constexpr (types::HASNORMALCOV<TConfig>) {
      // Set normals and covariances to points
      this->compute_normal_covariance(adjacent_voxels, std::min(num_neighbors, TConfig::NUM_NEIGHBORS), use_active_map);
    }
    // Update debug information (maintained only for active map)
    if (!use_active_map) return;
    this->debug_.points_added = counter;
    this->debug_.voxel_added = this->get_active_map().size() - this->debug_.num_voxel;
    this->debug_.num_voxel = this->get_active_map().size();
    this->debug_.num_points = this->num_points();
  }

  /**
   * @brief Update the map with new points
   * @param [in] points             Points to update
   * @param [in] pose               Pose the points are in
   * @param [in] map_density        Map density to use for adding points (either max points per
   * voxel or adaptive map density)
   * @param [in] adjacent_voxels    Number of adjacent voxels to search for
   * @param [in] num_neighbors      Number of neighbors to search for
   */
  void update_points(const std::vector<types::Point<TConfig>>& points, const Sophus::SE3f& pose,
    const std::variant<int16_t, types::AdaptiveMapDensity>& map_density = TConfig::MAX_POINTS_PER_VOXEL,
    const int16_t adjacent_voxels = 1, const int16_t num_neighbors = TConfig::NUM_NEIGHBORS) override
  {
    // Start timer
    auto start = std::chrono::high_resolution_clock::now();

    // Transform points to map frame and map point type
    std::vector<types::Point<types::Point_XYZ>> points_transformed(points.size());
    std::transform(points.cbegin(), points.cend(), points_transformed.begin(),
      [&](const auto& point) { return utils::convert_point<TConfig, types::Point_XYZ>(point); });
    utils::transform_points(pose, points_transformed);
    const Eigen::Vector3f& origin = pose.translation();

    // Add new points to the map
    this->add_points(points_transformed, map_density, adjacent_voxels, num_neighbors);

    // Recompute neighbors if normals and covariances are enabled
    // NOTE: needs to be done before removal, as otherwise the pointers
    // become invalid
    if constexpr (types::HASNORMALCOV<TConfig>) {
      // Set normals and covariances to points
      this->compute_normal_covariance(adjacent_voxels, std::min(num_neighbors, TConfig::NUM_NEIGHBORS));
    }

    // Remove far points from the map
    this->remove_far_points(origin);

    // Compute elapsed time
    // clang-format off
    double time = std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::high_resolution_clock::now() - start).count() * 1.0e-3;  // NOLINT
    // clang-format on
    // Update debug information
    this->debug_.update_time = time;
  }

  /**
   * @brief Get the map density
   * @param [in] range                Range to consider
   * @return                          Adaptive Map density
   */
  types::AdaptiveMapDensity get_density(double range)
  {
    double range2 = range * range;
    unsigned int voxels = 0;
    unsigned int points = 0;
    for (auto it = this->get_active_map().cbegin(); it != this->get_active_map().cend();) {
      const auto& [voxel, voxel_points] = *it;
      const auto& pt = voxel_points.front();
      if (pt.pos.squaredNorm() < range2) {
        ++voxels;
        points += voxel_points.size();
        ++it;
      } else {
        ++it;
      }
    }
    // clang-format off
    types::AdaptiveMapDensity density{};
    density.range = range;
    density.max_points = (voxels > 0) ? static_cast<std::int16_t>(points / voxels) : 0;  // NOLINT
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
  std::vector<types::Point<TConfig>> get_cloud() const override
  {
    std::vector<types::Point<TConfig>> points;
    points.reserve(this->get_active_map().size() * static_cast<size_t>(TConfig::MAX_POINTS_PER_VOXEL));
    for (auto it = this->get_active_map().cbegin(); it != this->get_active_map().cend(); ++it) {
      const auto& voxel_points = it.value();
      points.insert(points.end(), voxel_points.cbegin(), voxel_points.cend());
    }
    points.shrink_to_fit();
    return points;
  }

  /**
   * @brief Get the vector holding the neighbors of the points in the map
   * @return                        Neighbors of the points in the map
   * @note this is required for testing of the muliple nearest neighbor search
   */
  std::vector<types::Neighbors<TConfig>> get_neighbors() const override
  {
    return std::vector<types::Neighbors<TConfig>>(
      this->get_active_neighbors().cbegin(), this->get_active_neighbors().cend());
  }

  /**
   * @brief Switch the active map
   */
  void switch_active_map() override
  {
    // Switch active map
    this->map_a_active_.store(!this->map_a_active_.load());
    // Update debug information as not set in async load
    this->debug_.points_added = this->num_points();
    this->debug_.voxel_added = this->get_active_map().size();
    this->debug_.num_voxel = this->debug_.voxel_added;
    this->debug_.num_points = this->debug_.points_added;
  }

protected:
  // Inherit constructor from MapHandler for param manager and logger
  VoxelHashMap(tam::pmg::ParamReferenceManager* pmg, tam::tsl::ReferenceLogger* logger)
      : MapHandler<TConfig>(pmg, logger)
  {
    // Additional initialization
  }

  // Inherit constructor from MapHandler for config and debug object
  VoxelHashMap(const types::MapConfig& config, const types::MapDebug& debug) : MapHandler<TConfig>(config, debug)
  {
    // Additional initialization
  }

  /**
   * @brief search for closest neighbors of a point
   * @param [in] point              Point to search for
   * @param [in] adjacent_voxels    Number of adjacent voxels to search in
   * @param [in] num_neighbors      Number of neighbors to search for
   * @param [in] use_active_map     Flag to indicate if the active or inactive map should be used
   * @note The found neighbors are not sorted by distance
   * @note Function does not check whether num_neighbors exceeds TConfig::NUM_NEIGHBORS
   */
  void search_closest_neighbors(types::Neighbors<TConfig>& point, const int16_t adjacent_voxels,
    const int16_t num_neighbors, const bool use_active_map = true) const override
  {
    const MapType& target_map = use_active_map ? this->get_active_map() : this->get_inactive_map();
    // Convert the point to voxel coordinates
    const auto& voxel = point_to_voxel<TConfig>(*(point.point), this->config_.voxel_size);
    // Get nearby voxels on the map
    const auto& neighbor_voxels = get_adjacent_voxels(voxel, adjacent_voxels);
    // Iterate over the neighbor voxels
    std::vector<types::Point<TConfig>*> neighbors{};
    int16_t num_neighbor_voxels = (2 * adjacent_voxels + 1) * (2 * adjacent_voxels + 1) * (2 * adjacent_voxels + 1);
    neighbors.reserve(num_neighbor_voxels * TConfig::MAX_POINTS_PER_VOXEL);
    std::for_each(neighbor_voxels.cbegin(), neighbor_voxels.cend(), [&](const auto& neighbor) {
      auto search = target_map.find(neighbor);
      if (search != target_map.end()) {
        const auto& map_points = search.value();
        if (!map_points.empty()) {
          for (const auto& map_point : map_points) {
            neighbors.emplace_back(const_cast<types::Point<TConfig>*>(&map_point));
          }
        }
      }
    });
    if (!neighbors.empty()) {
      std::for_each(neighbors.cbegin(), neighbors.cend(), [&](const auto& neighbor) {
        float distance = (neighbor->pos - point.point->pos).norm();
        if (point.num_neighbors == num_neighbors) {
          // If we already have enough neighbors, check if the current one is closer
          float max_distance = 0.0f;
          int16_t largest_neighbor_index = -1;
          // iterate through all existing neighbors
          for (int16_t neighbor_idx = 0; neighbor_idx < num_neighbors; ++neighbor_idx) {
            // if current distance is larger than maximal_distance and distance
            // update
            if (max_distance < point.distance[neighbor_idx] && distance < point.distance[neighbor_idx]) {
              max_distance = point.distance[neighbor_idx];
              largest_neighbor_index = neighbor_idx;
            }
          }
          // replace largest neighbor with current one
          if (largest_neighbor_index != -1) {
            point.neighbor[largest_neighbor_index] = neighbor;
            point.distance[largest_neighbor_index] = distance;
          }
        } else {
          // If we have space for more neighbors, just add it
          point.neighbor[point.num_neighbors] = neighbor;
          point.distance[point.num_neighbors++] = distance;
        }
      });
    }
    return;
  }

private:
  /**
   * @brief Compute normals and covariances for neighbors multi-threaded
   * @param[in] adjacent_voxels             Adjacent voxels to search in
   * @param[in] num_neighbors               Number of neighbors to search for
   * @param[in] use_active_map              Flag to indicate if the active or inactive map should be
   * used
   */
  void compute_normal_covariance(
    const int16_t adjacent_voxels, const int16_t num_neighbors, const bool use_active_map = true)
    requires types::HASNORMALCOV<TConfig>
  {
    tbb::concurrent_vector<types::Neighbors<TConfig>>& target_neighbors =
      use_active_map ? this->get_active_neighbors() : this->get_inactive_neighbors();
    // Start timer
    auto start = std::chrono::high_resolution_clock::now();

    // Compute normals and covariances
    using neighbor_iterator = tbb::concurrent_vector<types::Neighbors<TConfig>>::iterator;
    tbb::parallel_for(tbb::blocked_range<neighbor_iterator>(target_neighbors.begin(), target_neighbors.end()),
      [&](const tbb::blocked_range<neighbor_iterator>& r) {
        std::for_each(r.begin(), r.end(), [&](auto& neighbor) {
          // Search for correspondences (= closest neighbors)
          this->search_closest_neighbors(neighbor, adjacent_voxels, num_neighbors, use_active_map);
          // Set normal and covariance to frame point (= map point)
          this->set_normal_covariance(neighbor, num_neighbors);
        });
      });
    // Compute elapsed time
    // clang-format off
    double time = std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::high_resolution_clock::now() - start).count() * 1.0e-3;  // NOLINT
    // clang-format on
    // Update debug information
    if (use_active_map) this->debug_.normal_cov_time = time;
  }

  /**
   * @brief Remove points that are too far from the origin
   * @param [in] origin              current origin
   */
  void remove_far_points(const Eigen::Vector3f& origin)
  {
    const double max_distance2 = this->config_.max_distance * this->config_.max_distance;
    unsigned int counter = 0;
    for (auto it = this->get_active_map().begin(); it != this->get_active_map().end();) {
      const auto& [voxel, voxel_points] = *it;
      const auto& pt = voxel_points.front();
      if ((pt.pos - origin).squaredNorm() >= max_distance2) {
        it = this->get_active_map().erase(it);
        counter++;
      } else {
        ++it;
      }
    }
    // Update debug information
    this->debug_.voxel_removed = counter;
    this->debug_.num_voxel = this->get_active_map().size();
    this->debug_.points_removed = this->num_points() - this->debug_.num_points;
    this->debug_.num_points = this->num_points();
  }

  /**
   * @brief Get active and inactive map references
   * @note Cannot be in base class as map types differs
   */
  // clang-format off
  MapType & get_active_map() { return this->map_a_active_.load() ? map_a_ : map_b_; }  // NOLINT
  const MapType & get_active_map() const { return this->map_a_active_.load() ? map_a_ : map_b_; }  // NOLINT
  size_t get_active_map_size() { return this->map_a_active_.load() ? map_a_.size() : map_b_.size(); }  // NOLINT
  tbb::concurrent_vector<types::Neighbors<TConfig>> & get_active_neighbors() { return this->map_a_active_.load() ? tbb_neighbors_a_ : tbb_neighbors_b_; }  // NOLINT
  const tbb::concurrent_vector<types::Neighbors<TConfig>> & get_active_neighbors() const { return this->map_a_active_.load() ? tbb_neighbors_a_ : tbb_neighbors_b_; }  // NOLINT
  MapType & get_inactive_map() { return this->map_a_active_.load() ? map_b_ : map_a_; }  // NOLINT
  const MapType & get_inactive_map() const { return this->map_a_active_.load() ? map_b_ : map_a_; }  // NOLINT
  size_t get_inactive_map_size() { return this->map_a_active_.load() ? map_b_.size() : map_a_.size(); }  // NOLINT
  tbb::concurrent_vector<types::Neighbors<TConfig>> & get_inactive_neighbors() { return this->map_a_active_.load() ? tbb_neighbors_b_ : tbb_neighbors_a_; }  // NOLINT
  const tbb::concurrent_vector<types::Neighbors<TConfig>> & get_inactive_neighbors() const { return this->map_a_active_.load() ? tbb_neighbors_b_ : tbb_neighbors_a_; }  // NOLINT

  // clang-format on

  /**
   * @brief Allocate map switching vectors
   * @param [in] update_map          Flag to indicate if the map is being updated
   * @param [in] size                Size to allocate
   */
  void allocate_memory([[maybe_unused]] const bool update_map, [[maybe_unused]] const size_t size) override {}

private:
  // Member variables
  MapType map_a_{};
  MapType map_b_{};
  tbb::concurrent_vector<types::Neighbors<TConfig>> tbb_neighbors_a_{};
  tbb::concurrent_vector<types::Neighbors<TConfig>> tbb_neighbors_b_{};
};
}  // namespace tam::core::state
