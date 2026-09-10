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
// modified from https://github.com/PRBonn/kiss-icp/blob/main/cpp/kiss_icp/core/Threshold.cpp
#pragma once

#include <algorithm>
#include <memory>
#include <sophus/se3.hpp>

#include "threshold_handler/threshold_handler_base.hpp"

namespace tam::core::state {
template <typename TConfig>
class AdaptiveThreshold : public ThresholdHandler<TConfig>
{
public:
  /**
   * @brief Constructor for param manager and logger
   */
  static std::unique_ptr<ThresholdHandler<TConfig>> from_config(
    tam::pmg::ParamReferenceManager* pmg, tam::tsl::ReferenceLogger* logger)
  {
    std::unique_ptr<AdaptiveThreshold<TConfig>> ct =
      std::unique_ptr<AdaptiveThreshold<TConfig>>(new AdaptiveThreshold<TConfig>(pmg, logger));
    return ct;
  }

  /**
   * @brief Constructor for config and debug objects
   */
  static std::unique_ptr<ThresholdHandler<TConfig>> from_config(
    const types::ThresholdConfig& config, const types::ThresholdDebug& debug)
  {
    std::unique_ptr<AdaptiveThreshold<TConfig>> ct =
      std::unique_ptr<AdaptiveThreshold<TConfig>>(new AdaptiveThreshold<TConfig>(config, debug));
    return ct;
  }

  /**
   * @brief Set the model deviation between initial guess and registration result
   */
  void set_model_deviation(
    [[maybe_unused]] const Sophus::SE3f& init_guess, [[maybe_unused]] const Sophus::SE3f& pose_registered) override
  {
    if (!this->initialized_) {
      // If not initialized, just initialize the module and return without updating the model error
      this->previous_init_guess_ = init_guess;
      this->initialized_ = true;
      return;
    }
    // Compute deviation between initial guess and registration
    const Sophus::SE3f model_deviation = init_guess.inverse() * pose_registered;
    // Compute model error
    const double model_error = [&]() {
      const double theta = Eigen::AngleAxisf(model_deviation.rotationMatrix()).angle();
      const double delta_rot = 2.0 * this->config_.max_correspondence_range * std::sin(theta / 2.0);
      const double delta_trans = model_deviation.translation().norm();
      return delta_trans + delta_rot;
    }();
    // Update the model error given the vehicle is actually moving translationally
    // and the model error is significant
    if (this->initialized_ &&
      (this->previous_init_guess_.inverse() * init_guess).translation().norm() >
        3 * this->config_.min_motion_threshold &&
      model_error > this->config_.min_motion_threshold) {
      this->model_sse_ += model_error * model_error;
      this->num_samples_++;
      this->debug_.model_error = model_error;
    }
  }

  /**
   * @brief Return the current adaptive threshold
   */
  double get_threshold() override
  {
    this->debug_.internal_threshold = std::sqrt(this->model_sse_ / this->num_samples_);
    this->debug_.current_threshold = std::min(this->debug_.internal_threshold, 10.0);
    return this->debug_.current_threshold;
  };

  /**
   * @brief Initialize member variables with params
   */
  void init() override { model_sse_ = this->config_.initial_threshold * this->config_.initial_threshold; }

protected:
  // Inherit constructor from ModelHandler for param manager and logger
  AdaptiveThreshold(tam::pmg::ParamReferenceManager* pmg, tam::tsl::ReferenceLogger* logger)
      : ThresholdHandler<TConfig>(pmg, logger)
  {
    // Additional initialization
    num_samples_ = 1;
  }

  // Inherit constructor from ModelHandler for config and debug object
  AdaptiveThreshold(const types::ThresholdConfig& config, const types::ThresholdDebug& debug)
      : ThresholdHandler<TConfig>(config, debug)
  {
    // Additional initialization
    num_samples_ = 1;
  }

private:
  bool initialized_{false};
  double model_sse_{};
  int num_samples_{};
  Sophus::SE3f previous_init_guess_{};
};
}  // namespace tam::core::state
