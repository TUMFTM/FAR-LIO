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
#include <memory>
#include <string>
#include <unordered_map>
#include <variant>
#include <vector>

#include "odometry_types/point_types.hpp"

namespace tam::core::state::types {
enum class MapType : std::uint8_t { VOXELHASHMAP = 0, CUDA_VOXELHASHMAP = 1 };
/**
 * @brief Covariance regularization type
 */
enum class CovRegularizationType : std::uint8_t { SVD = 0, FROBENIUS = 1, MIN_EIGENVALUE = 2 };

/**
 * @brief Map resolution
 */
struct AdaptiveMapDensity {
  bool valid{false};
  Eigen::Vector3f origin = Eigen::Vector3f::Zero();
  std::int16_t max_points{0};
  std::int16_t min_points{0};
  float range{0.0f};
  float max_points_scale{-0.05f};
};

/**
 * @brief Configuration for the map
 */
struct MapConfig {
  bool frame_map{false};  // true if the map is to be created from just one frame
  double voxel_size{0.0};
  double max_distance{0.0};
  std::string cov_regularization{"SVD"};  // Options: "SVD", "FROBENIUS", "MIN_EIGENVALUE"
};

/**
 * @brief Debug signals for the map
 */
struct MapDebug {
  std::int64_t num_voxel{0};
  std::int64_t num_points{0};
  std::int64_t points_added{0};    // used during dynamic map update
  std::int64_t points_removed{0};  // used during dynamic map update
  std::int64_t voxel_added{0};     // used during dynamic map update
  std::int64_t voxel_removed{0};   // used during dynamic map update
  double update_time{0.0};         // used during dynamic map update
  double normal_cov_time{0.0};
  double load_factor{0.0};
  std::unordered_map<std::string, std::variant<std::int64_t, double>> conditional{};
};
}  // namespace tam::core::state::types
