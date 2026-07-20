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
#include <map>
#include <memory>
#include <string>
#include <vector>

#include "odometry_types/odometry_types.hpp"

namespace tam::core::state::types {
enum class CovarianceType : std::uint8_t {
  CONSTANT = 0,
  CENSI = 1,
  ERRORDISTRIBUTIONP2P = 2,
  ERRORDISTRIBUTIONP2PL = 3,
  SEGWEIGHTEDP2P = 4,
  SEGWEIGHTEDP2PL = 5
};

/**
 * @brief Configuration for the covariance handler
 */
struct CovarianceConfig {
  std::vector<double> min_cov_translation{0.01, 0.01, 0.01};
  std::vector<double> min_cov_orientation{0.001, 0.001, 0.001};
  std::vector<double> min_cov_linear_twist{0.1, 0.1, 0.1};
  std::vector<double> min_cov_angular_twist{0.01, 0.01, 0.01};
};

/**
 * @brief Debug signals for the covariance handler
 */
struct CovarianceDebug {
  double pose_covariance_time{0.0};
  double twist_covariance_time{0.0};
};
}  // namespace tam::core::state::types
