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

#include <algorithm>
#include <memory>
#include <sophus/se3.hpp>

#include "threshold_handler/threshold_handler_base.hpp"

namespace tam::core::state {
template <typename TConfig>
class FixedThreshold : public ThresholdHandler<TConfig>
{
public:
  /**
   * @brief Constructor for param manager and logger
   */
  static std::unique_ptr<ThresholdHandler<TConfig>> from_config(
    tam::pmg::ParamReferenceManager* pmg, tam::tsl::ReferenceLogger* logger)
  {
    std::unique_ptr<FixedThreshold<TConfig>> th =
      std::unique_ptr<FixedThreshold<TConfig>>(new FixedThreshold<TConfig>(pmg, logger));
    return th;
  }

  /**
   * @brief Constructor for config and debug objects
   */
  static std::unique_ptr<ThresholdHandler<TConfig>> from_config(
    const types::ThresholdConfig& config, const types::ThresholdDebug& debug)
  {
    std::unique_ptr<FixedThreshold<TConfig>> th =
      std::unique_ptr<FixedThreshold<TConfig>>(new FixedThreshold<TConfig>(config, debug));
    return th;
  }

  /**
   * @brief Set the model deviation between initial guess and registration result
   */
  void set_model_deviation(
    [[maybe_unused]] const Sophus::SE3f& init_guess, [[maybe_unused]] const Sophus::SE3f& pose_registered) override
  {
    return;
  }

  /**
   * @brief Return the fixed threshold
   */
  double get_threshold() override { return this->config_.initial_threshold; }

  /**
   * @brief Initialize member variables with params
   */
  void init() override { return; }

protected:
  // Inherit constructor from ModelHandler for param manager and logger
  FixedThreshold(tam::pmg::ParamReferenceManager* pmg, tam::tsl::ReferenceLogger* logger)
      : ThresholdHandler<TConfig>(pmg, logger)
  {
    // Additional initialization
  }

  // Inherit constructor from ModelHandler for config and debug object
  FixedThreshold(const types::ThresholdConfig& config, const types::ThresholdDebug& debug)
      : ThresholdHandler<TConfig>(config, debug)
  {
    // Additional initialization
  }

private:
};
}  // namespace tam::core::state
