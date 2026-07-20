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
#include <eigen3/Eigen/Core>
#include <map>
#include <memory>

namespace tam::core::state::types {
struct Empty {
};

/**
 * @brief Point type for the map
 */
template <typename TConfig>
struct Point {
  // Geometrical coordinates
  Eigen::Vector3f pos{Eigen::Vector3f::Zero()};
  // Original sensor-frame spherical coordinates
  // Values stay untransformed to maintain the true line-of-sight
  std::conditional_t<TConfig::SPHERICAL, float, tam::core::state::types::Empty> azimuth{};
  std::conditional_t<TConfig::SPHERICAL, float, tam::core::state::types::Empty> range{};
  std::conditional_t<TConfig::SPHERICAL, float, tam::core::state::types::Empty> elevation{};
  // Point-wise timestamp relative to the message stamp
  float timestamp{};
  // Intensity
  std::conditional_t<TConfig::INTENSITY, float, tam::core::state::types::Empty> intensity{};
  // Normals
  std::conditional_t<TConfig::NORMALS, Eigen::Vector3f, tam::core::state::types::Empty> normal{};
  // Covariance
  std::conditional_t<TConfig::COV, Eigen::Matrix3f, tam::core::state::types::Empty> cov{};
  // Segmentation
  std::conditional_t<TConfig::SEG, std::uint32_t, tam::core::state::types::Empty> seg{};
  // Velocity
  std::conditional_t<TConfig::VEL, float, tam::core::state::types::Empty> vel{};
  // Radar Cross Section (RCS)
  std::conditional_t<TConfig::RCS, float, tam::core::state::types::Empty> rcs{};
  // SNR: Signal to Noise Ratio
  std::conditional_t<TConfig::SNR, float, tam::core::state::types::Empty> snr{};
  // Radar Confidence
  std::conditional_t<TConfig::CONFIDENCE, float, tam::core::state::types::Empty> confidence{};
  // Velocity Interval
  std::conditional_t<TConfig::VEL_INTERVAL, float, tam::core::state::types::Empty> vel_interval{};
  // Sensor id
  std::conditional_t<TConfig::SENSOR_ID, std::uint8_t, tam::core::state::types::Empty> sensor_id{};
};

/**
 * @brief Correspondence definition
 */
template <typename TConfig>
struct Correspondence {
  Correspondence() = default;
  types::Point<TConfig> frame{};
  types::Point<TConfig> map{};
  // Euclidean distance
  float distance{};
  // Precision matrix
  std::conditional_t<TConfig::COV, Eigen::Matrix3f, tam::core::state::types::Empty> precision{};
};

/**
 * @brief Neighbors Definition for the map
 */
template <typename TConfig>
struct Neighbors {
  Neighbors() = default;
  types::Point<TConfig>* point{};
  types::Point<TConfig>* neighbor[TConfig::NUM_NEIGHBORS] = {};
  float distance[TConfig::NUM_NEIGHBORS] = {};
  std::uint8_t num_neighbors{};
};
}  // namespace tam::core::state::types
