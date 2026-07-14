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

#include <chrono>
#include <deque>
#include <eigen3/Eigen/Core>
#include <iostream>
#include <memory>
#include <optional>
#include <sophus/se3.hpp>
#include <vector>

#include "model_handler/model_utils.hpp"
#include "odometry_base/odometry_base.hpp"
#include "odometry_types/odometry_types.hpp"
namespace tam::core::state
{
template <typename TConfig>
class ModelHandler : public OdometryBase<TConfig, types::ModelConfig, types::ModelDebug>
{
public:
  /**
   * @brief Set pose from external source
   * @param [in] pose               Pose to set
   * @param [in] valid              Whether the given pose is valid
   */
  virtual void set_pose(const types::PoseStamped & pose, const bool valid) = 0;
  /**
   * @brief Get the initial guess
   * @param [in] stamp              Timestamp to get the initial guess for
   * @return Initial guess
   */
  virtual types::PoseStamped get_initial_guess([[maybe_unused]] std::uint64_t stamp) = 0;
  /**
   * @brief Initialize current guess pose from config
   */
  void init_model_config()
  {
    this->current_guess_ = types::PoseStamped{};
    this->current_guess_.value().pose = Sophus::SE3f(
      Sophus::SE3f::QuaternionType(
        this->config_.initial_rot_w, this->config_.initial_rot_x, this->config_.initial_rot_y,
        this->config_.initial_rot_z),
      Sophus::SE3f::Point(
        this->config_.initial_pos_x, this->config_.initial_pos_y, this->config_.initial_pos_z));
    this->current_guess_.value().stamp =
      std::chrono::system_clock::now().time_since_epoch().count();
  }

protected:
  // Inherit constructor from OdometryBase for param manager and logger
  ModelHandler(tam::pmg::ParamReferenceManager * pmg, tam::tsl::ReferenceLogger * logger)
  : OdometryBase<TConfig, types::ModelConfig, types::ModelDebug>(pmg, logger)
  {
    this->set_config(pmg);
    this->set_logging(logger);
    // Additional initialization
  }
  // Inherit constructor from OdometryBase for config and debug object
  ModelHandler(const types::ModelConfig & config, const types::ModelDebug & debug)
  : OdometryBase<TConfig, types::ModelConfig, types::ModelDebug>(config, debug)
  {
    // Additional initialization
  }
  /**
   * @brief Set the configuration of the model handler from the param manager
   * @param [in] pmg                Param manager
   */
  void set_config(tam::pmg::ParamReferenceManager * pmg) override
  {
    // clang-format off
    pmg->declare_parameter("model.initial_pos_x", &this->config_.initial_pos_x, 0.0, tam::pmg::ParameterType::DOUBLE, "Initial pose: x position (m)"); // NOLINT
    pmg->declare_parameter("model.initial_pos_y", &this->config_.initial_pos_y, 0.0, tam::pmg::ParameterType::DOUBLE, "Initial pose: y position (m)"); // NOLINT
    pmg->declare_parameter("model.initial_pos_z", &this->config_.initial_pos_z, 0.0, tam::pmg::ParameterType::DOUBLE, "Initial pose: z position (m)"); // NOLINT
    pmg->declare_parameter("model.initial_rot_x", &this->config_.initial_rot_x, 0.0, tam::pmg::ParameterType::DOUBLE, "Initial pose: quaternion x"); // NOLINT
    pmg->declare_parameter("model.initial_rot_y", &this->config_.initial_rot_y, 0.0, tam::pmg::ParameterType::DOUBLE, "Initial pose: quaternion y"); // NOLINT
    pmg->declare_parameter("model.initial_rot_z", &this->config_.initial_rot_z, 0.0, tam::pmg::ParameterType::DOUBLE, "Initial pose: quaternion z"); // NOLINT
    pmg->declare_parameter("model.initial_rot_w", &this->config_.initial_rot_w, 1.0, tam::pmg::ParameterType::DOUBLE, "Initial pose: quaternion w"); // NOLINT
    // clang-format on
  }
  /**
   * @brief Register the debug variables with the logger
   * @param [in] logger             Logger
   */
  void set_logging(tam::tsl::ReferenceLogger * logger) const override
  {
    logger->log("model/current_pos_x", &this->debug_.current_pos_x);
    logger->log("model/current_pos_y", &this->debug_.current_pos_y);
    logger->log("model/current_pos_z", &this->debug_.current_pos_z);
    logger->log("model/current_rot_x", &this->debug_.current_rot_x);
    logger->log("model/current_rot_y", &this->debug_.current_rot_y);
    logger->log("model/current_rot_z", &this->debug_.current_rot_z);
    logger->log("model/current_rot_w", &this->debug_.current_rot_w);
  }

protected:
  std::optional<types::PoseStamped> current_guess_{};
  std::optional<types::PoseStamped> previous_guess_{};
};
}  // namespace tam::core::state
