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

#include "model_handler/external_guess.hpp"

#include <chrono>
#include <iostream>
#include <memory>

#include "param_management_cpp/param_reference_manager.hpp"
#include "tsl_logger_cpp/reference_logger.hpp"
int main()
{
  // clang-format off
  // Construct from param manager and logger
  tam::pmg::ParamReferenceManager::UniquePtr pmg_ =  std::make_unique<tam::pmg::ParamReferenceManager>(); // NOLINT
  tam::tsl::ReferenceLogger::UniquePtr logger_ = std::make_unique<tam::tsl::ReferenceLogger>(); // NOLINT
  std::unique_ptr<tam::core::state::ModelHandler<tam::core::state::types::ICP_EXT>> model_ =
    tam::core::state::ExternalGuess<tam::core::state::types::ICP_EXT>::from_config(pmg_.get(), logger_.get()); // NOLINT

  std::uint64_t stamp = std::chrono::system_clock::now().time_since_epoch().count();
  const Sophus::SE3f pose = Sophus::SE3f();
  model_->set_pose(tam::core::state::types::PoseStamped{pose, stamp}, true);  // NOLINT
  // clang-format on
  std::cout << "Initial position: (" << model_->get_initial_guess(stamp).pose.translation().x()
            << ", " << model_->get_initial_guess(stamp).pose.translation().y() << ", "
            << model_->get_initial_guess(stamp).pose.translation().z() << ")" << std::endl;
  return 0;
}
