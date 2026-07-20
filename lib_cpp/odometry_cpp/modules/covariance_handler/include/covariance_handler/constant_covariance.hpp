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
// Implementation strongly based on
#pragma once

#include <chrono>
#include <memory>
#include <vector>

#include "covariance_handler/covariance_handler_base.hpp"

namespace tam::core::state {
template <typename TConfig>
class ConstantCovariance : public CovarianceHandler<TConfig>
{
public:
  /**
   * @brief Constructor for param manager and logger
   */
  static std::unique_ptr<CovarianceHandler<TConfig>> from_config(
    tam::pmg::ParamReferenceManager* pmg, tam::tsl::ReferenceLogger* logger)
  {
    std::unique_ptr<ConstantCovariance<TConfig>> ch =
      std::unique_ptr<ConstantCovariance<TConfig>>(new ConstantCovariance<TConfig>(pmg, logger));
    return ch;
  }

  /**
   * @brief Constructor for config and debug objects
   */
  static std::unique_ptr<CovarianceHandler<TConfig>> from_config(
    const types::CovarianceConfig& config, const types::CovarianceDebug& debug)
  {
    std::unique_ptr<ConstantCovariance<TConfig>> ch =
      std::unique_ptr<ConstantCovariance<TConfig>>(new ConstantCovariance<TConfig>(config, debug));
    return ch;
  }

  std::array<float, 36> get_pose_covariance([[maybe_unused]] const Sophus::SE3f& pose,
    [[maybe_unused]] const std::vector<types::Correspondence<TConfig>>& correspondences,
    [[maybe_unused]] const float kernel_scale) override
  {
    auto start = std::chrono::high_resolution_clock::now();
    Eigen::Matrix<float, 6, 6> cov = Eigen::Matrix<float, 6, 6>::Zero();
    // clang-format off
    this->debug_.pose_covariance_time = std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::high_resolution_clock::now() - start).count() * 1.0e-3;  // NOLINT
    // clang-format on
    std::array<float, 6> min_cov{static_cast<float>(this->config_.min_cov_translation[0]),
      static_cast<float>(this->config_.min_cov_translation[1]),
      static_cast<float>(this->config_.min_cov_translation[2]),
      static_cast<float>(this->config_.min_cov_orientation[0]),
      static_cast<float>(this->config_.min_cov_orientation[1]),
      static_cast<float>(this->config_.min_cov_orientation[2])};
    return this->matrix2array(cov, min_cov);
  }

protected:
  // Inherit constructor from MapHandler for param manager and logger
  ConstantCovariance(tam::pmg::ParamReferenceManager* pmg, tam::tsl::ReferenceLogger* logger)
      : CovarianceHandler<TConfig>(pmg, logger)
  {
    // Additional initialization
  }

  // Inherit constructor from MapHandler for config and debug object
  ConstantCovariance(const types::CovarianceConfig& config, const types::CovarianceDebug& debug)
      : CovarianceHandler<TConfig>(config, debug)
  {
    // Additional initialization
  }
};
}  // namespace tam::core::state
