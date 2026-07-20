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

#include "threshold_handler/adaptive_threshold.hpp"

#include <iostream>
#include <sophus/se3.hpp>

#include "param_management_cpp/param_reference_manager.hpp"
#include "tsl_logger_cpp/reference_logger.hpp"

int main()
{
  // clang-format off
  // Construct from param manager and logger
  tam::pmg::ParamReferenceManager::UniquePtr pmg_ =  std::make_unique<tam::pmg::ParamReferenceManager>(); // NOLINT
  tam::tsl::ReferenceLogger::UniquePtr logger_ = std::make_unique<tam::tsl::ReferenceLogger>(); // NOLINT
  std::unique_ptr<tam::core::state::ThresholdHandler<tam::core::state::types::ICP_EXT>> threshold_ =  // NOLINT
    tam::core::state::AdaptiveThreshold<tam::core::state::types::ICP_EXT>::from_config(pmg_.get(), logger_.get()); // NOLINT
  // clang-format on

  const Sophus::SE3f model_deviation =
    Sophus::SE3f(Sophus::SE3f::QuaternionType(1.0, 0.0, 0.0, 0.0), Sophus::SE3f::Point(1.0, 2.0, 3.0));
  threshold_->set_model_deviation(model_deviation, Sophus::SE3f());

  std::cout << "Threshold: " << threshold_->get_threshold() << std::endl;
  return 0;
}
