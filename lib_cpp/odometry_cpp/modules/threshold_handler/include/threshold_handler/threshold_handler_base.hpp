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
#include <sophus/se3.hpp>

#include "odometry_base/odometry_base.hpp"

namespace tam::core::state {
template <typename TConfig>
class ThresholdHandler : public OdometryBase<TConfig, types::ThresholdConfig, types::ThresholdDebug>
{
public:
  virtual void init() = 0;
  virtual void set_model_deviation(
    [[maybe_unused]] const Sophus::SE3f& init_guess, [[maybe_unused]] const Sophus::SE3f& pose_registered) = 0;
  virtual double get_threshold() = 0;

protected:
  // Inherit constructor from OdometryBase for param manager and logger
  ThresholdHandler(tam::pmg::ParamReferenceManager* pmg, tam::tsl::ReferenceLogger* logger)
      : OdometryBase<TConfig, types::ThresholdConfig, types::ThresholdDebug>(pmg, logger)
  {
    this->set_config(pmg);
    this->set_logging(logger);
    // Additional initialization
  }

  // Inherit constructor from OdometryBase for config and debug object
  ThresholdHandler(const types::ThresholdConfig& config, const types::ThresholdDebug& debug)
      : OdometryBase<TConfig, types::ThresholdConfig, types::ThresholdDebug>(config, debug)
  {
  }

  /**
   * @brief Set the configuration of the threshold handler from the param manager
   * @param [in] pmg                Param manager
   */
  void set_config(tam::pmg::ParamReferenceManager* pmg) override
  {
    // clang-format off
    pmg->declare_parameter("threshold.initial_threshold", &this->config_.initial_threshold, 1.0, tam::pmg::ParameterType::DOUBLE, "Initial correspondence threshold (m)"); // NOLINT
    pmg->declare_parameter("threshold.min_motion_threshold", &this->config_.min_motion_threshold, 5.0, tam::pmg::ParameterType::DOUBLE, "Minimum motion to update the model error (m)"); // NOLINT
    pmg->declare_parameter("threshold.max_correspondence_range", &this->config_.max_correspondence_range, 100.0, tam::pmg::ParameterType::DOUBLE, "Max point range to convert rotational to translational error (m)"); // NOLINT
    // clang-format on
  }

  /**
   * @brief Register the debug variables with the logger
   * @param [in] logger             Logger
   */
  void set_logging(tam::tsl::ReferenceLogger* logger) const override
  {
    logger->log("threshold/internal_threshold", &this->debug_.internal_threshold);
    logger->log("threshold/current_threshold", &this->debug_.current_threshold);
    logger->log("threshold/model_error", &this->debug_.model_error);
  }
};
}  // namespace tam::core::state
