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

#include <deque>
#include <iostream>
#include <memory>
#include <sophus/se3.hpp>

#include "model_handler/model_handler_base.hpp"

namespace tam::core::state {
template <typename TConfig>
class ConstantVelocity : public ModelHandler<TConfig>
{
public:
  /**
   * @brief Constructor for param manager and logger
   */
  static std::unique_ptr<ModelHandler<TConfig>> from_config(
    tam::pmg::ParamReferenceManager* pmg, tam::tsl::ReferenceLogger* logger)
  {
    std::unique_ptr<ConstantVelocity<TConfig>> pm =
      std::unique_ptr<ConstantVelocity<TConfig>>(new ConstantVelocity<TConfig>(pmg, logger));
    return pm;
  }

  /**
   * @brief Constructor for config and debug objects
   */
  static std::unique_ptr<ModelHandler<TConfig>> from_config(
    const types::ModelConfig& config, const types::ModelDebug& debug)
  {
    std::unique_ptr<ConstantVelocity<TConfig>> pm =
      std::unique_ptr<ConstantVelocity<TConfig>>(new ConstantVelocity<TConfig>(config, debug));
    return pm;
  }

  /**
   * @brief Set pose from the pipeline after registration
   * @param [in] pose               Pose to set
   * @param [in] valid              Whether the given pose is valid
   */
  void set_pose(const types::PoseStamped& pose, const bool valid) override
  {
    if (!valid) {
      std::cerr << "Invalid pose received!" << std::endl;
      return;
    }
    if (!this->current_guess_.has_value()) {
      std::cerr << "Model not initialized, initializing to config!" << std::endl;
      this->init_model_config();
    } else {
      this->previous_guess_ = this->current_guess_;
      // Compute new pose predicting the difference between to the last pose forward
      Sophus::SE3f predicted_pose = utils::cv_prediction(pose.pose, this->current_guess_.value().pose);
      // Update current pose
      this->current_guess_.value().pose = predicted_pose;
      this->current_guess_.value().stamp = pose.stamp;
    }
  };

  /**
   * @brief Return current initial guess
   */
  types::PoseStamped get_initial_guess([[maybe_unused]] std::uint64_t stamp) override
  {
    if (!this->current_guess_.has_value()) {
      std::cerr << "Prediction model not initialized! - Return empty pose!" << std::endl;
      return types::PoseStamped{};
    }
    this->debug_.current_pos_x = this->current_guess_.value().pose.translation().x();
    this->debug_.current_pos_y = this->current_guess_.value().pose.translation().y();
    this->debug_.current_pos_z = this->current_guess_.value().pose.translation().z();
    this->debug_.current_rot_w = this->current_guess_.value().pose.so3().unit_quaternion().w();
    this->debug_.current_rot_x = this->current_guess_.value().pose.so3().unit_quaternion().x();
    this->debug_.current_rot_y = this->current_guess_.value().pose.so3().unit_quaternion().y();
    this->debug_.current_rot_z = this->current_guess_.value().pose.so3().unit_quaternion().z();
    return this->current_guess_.value();
  };

protected:
  // Inherit constructor from ModelHandler for param manager and logger
  ConstantVelocity(tam::pmg::ParamReferenceManager* pmg, tam::tsl::ReferenceLogger* logger)
      : ModelHandler<TConfig>(pmg, logger)
  {
    // Additional initialization
  }

  // Inherit constructor from ModelHandler for config and debug object
  ConstantVelocity(const types::ModelConfig& config, const types::ModelDebug& debug)
      : ModelHandler<TConfig>(config, debug)
  {
    // Additional initialization
  }
};
}  // namespace tam::core::state
