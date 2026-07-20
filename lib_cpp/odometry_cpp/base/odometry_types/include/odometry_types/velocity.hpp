/*
 * Copyright 2026 Oscar Breiner, Maximilian Leitenstern
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
#include <cstdint>
#include <limits>
#include <string>
#include <unordered_map>
#include <variant>
#include <vector>

namespace tam::core::state::types {
enum class VelocityType : std::uint8_t { DERIVATIVE = 0, DCR = 1 };

/**
 * @brief Single DCR hypothesis, i.e. a single aliasing wrap k for a single cluster
 */
struct DCRHypothesis {
  std::int16_t raw_cluster_idx{-1};
  std::int8_t k{0};
  std::vector<std::size_t> point_indices;
  std::vector<float> az_cos;
  std::vector<float> az_sin;
  std::vector<float> corr;
  Eigen::Vector2f v{0.0f, 0.0f};
  std::size_t n_points{0};
};

/**
 * @brief DCR cluster (raw DBSCAN cluster on (az, vel))
 */
struct DCRCluster {
  std::int16_t raw_cluster_idx{-1};
  std::int8_t sensor_id{-1};
  std::vector<std::size_t> point_indices;
  std::vector<float> az;
  std::vector<float> vel;
  std::vector<float> vint;
  std::vector<std::int8_t> k_all;
};

/**
 * @brief Result of the hypothesis selection step in DCR
 */
struct DCRHypothesisSelection {
  bool valid{false};
  float score{-std::numeric_limits<float>::infinity()};
  Eigen::Vector2f model_v{0.0f, 0.0f};
  std::vector<types::DCRHypothesis> hypotheses;
  std::size_t n_hits{0};
  std::size_t n_outliers{0};
};

/**
 * @brief Velocity fit result for a single candidate model in DCR RANSAC
 */
struct DCRVelocityFit {
  Eigen::Vector2f v{0.0f, 0.0f};
  std::vector<std::uint32_t> inlier;
  std::size_t n_inliers{0};
};

/**
 * @brief DCR (Doppler Consensus Reconstruction) tunables.
 */
struct DCRConfig {
  // Cluster (DBSCAN on (az, vel))
  double cluster_az_scale{1.0};
  double cluster_vel_scale{5.0};
  double cluster_eps{0.9};
  std::int64_t cluster_min_samples{5};
  // Candidate / RANSAC
  std::int64_t min_cluster_points{2};
  std::int64_t local_ransac_min_points{12};
  std::int64_t local_ransac_iterations{40};
  double local_ransac_inlier_threshold{0.8};
  std::int64_t local_ransac_sample_size{2};
  std::int64_t final_ransac_iterations{40};
  double final_ransac_inlier_threshold{1.0};
  double selection_hit_threshold{2.6};
  std::int64_t min_window_points{10};
};

/**
 * @brief Configuration for the velocity handler.
 */
struct VelocityConfig {
  DCRConfig dcr{};
};

/**
 * @brief Debug signals for the velocity handler.
 */
struct VelocityDebug {
  double velocity_time{0.0};  // [ms]
  std::unordered_map<std::string, std::variant<std::int64_t, double>> conditional{};
};
}  // namespace tam::core::state::types
