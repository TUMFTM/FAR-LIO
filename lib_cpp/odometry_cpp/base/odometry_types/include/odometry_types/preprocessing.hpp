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

#include <limits>
#include <numbers>
#include <string>
#include <unordered_map>
#include <vector>
namespace tam::core::state::types
{
enum class PreprocessingType : std::uint8_t { LIDAR = 0, CUDA_LIDAR = 1, RADAR = 2 };

enum class JitterFilterMethod {
  None = 0,
  NearestNeighbour = 1,
  NearestNeighbour80m = 2,
  AdaptiveNearestNeighbour = 3
};
struct RadarPreprocessingConfig
{
  // CROP in spherical coordinates (min/max elevation/azimuth)
  std::vector<double> crop_spherical{
    -std::numbers::pi, std::numbers::pi,           // azimuth
    -std::numbers::pi / 2, std::numbers::pi / 2};  // elevation
  // Min/max filtering for RCS
  std::vector<double> rcs_threshold{
    -std::numeric_limits<double>::infinity(), std::numeric_limits<double>::infinity()};
  // Min/max filtering for SNR
  std::vector<double> snr_threshold{
    -std::numeric_limits<double>::infinity(), std::numeric_limits<double>::infinity()};
  // Min/max filtering for velocity
  std::vector<double> velocity_threshold{
    -std::numeric_limits<double>::infinity(), std::numeric_limits<double>::infinity()};
  // Min/max filtering for confidence
  std::vector<double> confidence_threshold{
    -std::numeric_limits<double>::infinity(), std::numeric_limits<double>::infinity()};
  // Jitter filter method - persistent string storage for parameter binding
  std::string jitter_method = "None";
  double nn_radius = 3.0;                 // constant radius for all distances
  double relaxation_coefficient_t = 1.0;  // relaxation coefficient t
  std::int64_t max_min_neighbors = 20;    // max value for min_neighbors
  std::int64_t min_min_neighbors = 1;     // min value for min_neighbors (lower bound)
};
/**
 * @brief Configuration for the preprocessing
 */
struct PreprocessingConfig
{
  std::vector<double> crop_range{0.0, 200.0};
  // VEHICLE FOOTPRINT FILTER (exclude points within vehicle body)
  std::vector<double> crop_footprint{6.0, 1.0};  // longitudinal, lateral half-extent
  RadarPreprocessingConfig radar{};
};
/**
 * @brief Debug signals for the preprocessing
 */
struct PreprocessingDebug
{
  std::unordered_map<std::string, std::int64_t> num_points_preprocessed{};
  std::unordered_map<std::string, double> preprocess_time{};
};
}  // namespace tam::core::state::types
