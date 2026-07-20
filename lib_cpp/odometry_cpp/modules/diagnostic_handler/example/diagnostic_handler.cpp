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

#include <iostream>
#include <memory>

#include "diagnostic_handler/diagnostic_handler_base.hpp"
#include "param_management_cpp/param_reference_manager.hpp"
#include "tsl_logger_cpp/reference_logger.hpp"

int main()
{
  // clang-format off
  // Construct from param manager and logger
  tam::pmg::ParamReferenceManager::UniquePtr pmg_ =  std::make_unique<tam::pmg::ParamReferenceManager>(); // NOLINT
  tam::tsl::ReferenceLogger::UniquePtr logger_ = std::make_unique<tam::tsl::ReferenceLogger>(); // NOLINT
  std::unique_ptr<tam::core::state::DiagnosticHandler<tam::core::state::types::ICP_EXT>> diagnostic_handler_ =  // NOLINT
    tam::core::state::DiagnosticHandler<tam::core::state::types::ICP_EXT>::from_config(pmg_.get(), logger_.get()); // NOLINT
  // clang-format on

  // Create pose and initial guess
  const Sophus::SE3f pose =
    Sophus::SE3f(Sophus::SE3f::QuaternionType(1.0, 0.0, 0.0, 0.0), Sophus::SE3f::Point(1.0, 2.0, 3.0));
  const Sophus::SE3f init_guess =
    Sophus::SE3f(Sophus::SE3f::QuaternionType(1.0, 0.0, 0.0, 0.0), Sophus::SE3f::Point(2.0, 2.0, 3.0));
  // Create timestamps
  const std::uint64_t pose_stamp = 19483583754837;
  const std::uint64_t init_guess_stamp = 19483583754845;
  tam::core::state::types::PoseStamped pose_stamped = {pose, pose_stamp};
  tam::core::state::types::TangentStamped tangent_stamped = {Sophus::SE3f::Tangent(), pose_stamp};
  tam::core::state::types::PoseStamped init_guess_stamped = {init_guess, init_guess_stamp};
  // Create registration status object
  const tam::core::state::types::RegistrationStatus reg_status = {true, 50.0, 25};
  const tam::core::state::types::DiagnosticStatus status =
    diagnostic_handler_->get_diagnostic_status(pose_stamped, tangent_stamped, init_guess_stamped, reg_status);

  // Print status
  std::cout << "Level: " << static_cast<int>(status.level) << std::endl;
  std::cout << "Message: " << status.message << std::endl;
  for (const auto& [key, value] : status.key_values) {
    std::cout << key << ": " << value << std::endl;
  }
  return 0;
}
