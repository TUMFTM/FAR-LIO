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

#include <gtest/gtest.h>

#include <iostream>
#include <memory>

#include "diagnostic_handler/diagnostic_handler_base.hpp"
#include "param_management_cpp/param_reference_manager.hpp"
#include "tsl_logger_cpp/reference_logger.hpp"

/**
 * @brief Test construction of DiagnosticHandler from param manager and logger
 */
TEST(DiagnosticHandler, BuildPmgLogger)
{
  // clang-format off
  // Construct from param manager and logger
  tam::pmg::ParamReferenceManager::UniquePtr pmg_ =  std::make_unique<tam::pmg::ParamReferenceManager>(); // NOLINT
  tam::tsl::ReferenceLogger::UniquePtr logger_ = std::make_unique<tam::tsl::ReferenceLogger>(); // NOLINT
  std::unique_ptr<tam::core::state::DiagnosticHandler<tam::core::state::types::ICP_EXT>> diagnostic_handler_ =  // NOLINT
    tam::core::state::DiagnosticHandler<tam::core::state::types::ICP_EXT>::from_config(pmg_.get(), logger_.get()); // NOLINT
  // clang-format on
  tam::pmg::MgmtInterface* pmg_raw = pmg_.get();
  pmg_raw->set_value("diagnostic.ellipsis_size_s", 6.0);

  EXPECT_EQ(diagnostic_handler_->get_config().ellipsis_size_s, 6.0)
    << "Failed to construct DiagnosticHandler from param manager and logger";
}

/**
 * @brief Test construction of DiagnosticHandler from config and debug object
 */
TEST(DiagnosticHandler, BuildConfigDebug)
{
  // clang-format off
  // Construct from config and debug objects
  tam::core::state::types::DiagnosticConfig config;
  tam::core::state::types::DiagnosticDebug debug;
  config.ellipsis_size_s = 6.0;
  std::unique_ptr<tam::core::state::DiagnosticHandler<tam::core::state::types::ICP_EXT>> diagnostic_handler_ = // NOLINT
    tam::core::state::DiagnosticHandler<tam::core::state::types::ICP_EXT>::from_config(config, debug); // NOLINT
  // clang-format on
  EXPECT_EQ(diagnostic_handler_->get_config().ellipsis_size_s, 6.0)
    << "Failed to construct DiagnosticHandler from config and debug object";
}

/**
 * @brief Test DiagnosticHandler check of the input pointcloud status
 */
TEST(DiagnosticHandler, InputStatus)
{
  // clang-format off
  // Construct from config and debug objects
  tam::core::state::types::DiagnosticConfig config;
  tam::core::state::types::DiagnosticDebug debug;
  config.check_input_status = true;
  config.check_forward = false;
  config.check_ellipsis = false;
  config.check_time_diff = false;
  config.enable_vel = true;
  config.check_vel = false;
  std::unique_ptr<tam::core::state::DiagnosticHandler<tam::core::state::types::ICP_EXT>> diagnostic_handler_ = // NOLINT
    tam::core::state::DiagnosticHandler<tam::core::state::types::ICP_EXT>::from_config(config, debug); // NOLINT
  // clang-format on

  // Create pose and initial guess
  const Sophus::SE3f pose =
    Sophus::SE3f(Sophus::SE3f::QuaternionType(1.0, 0.0, 0.0, 0.0), Sophus::SE3f::Point(2.2, 2.0, 3.0));
  const Sophus::SE3f init_guess =
    Sophus::SE3f(Sophus::SE3f::QuaternionType(1.0, 0.0, 0.0, 0.0), Sophus::SE3f::Point(2.0, 2.0, 3.0));
  // Create timestamps
  const std::uint64_t pose_stamp = 19483583754837;
  const std::uint64_t init_guess_stamp = 19483583754837;
  tam::core::state::types::PoseStamped pose_stamped = {pose, pose_stamp};
  tam::core::state::types::TangentStamped tangent_stamped = {Sophus::SE3f::Tangent(), pose_stamp};
  tam::core::state::types::PoseStamped init_guess_stamped = {init_guess, init_guess_stamp};
  // Create registration status and input status object
  tam::core::state::types::RegistrationStatus reg_status{false, 250.0, 450};
  tam::core::state::types::DiagnosticStatus input_status{
    tam::core::state::types::DiagnosticLevel::ERROR, "Preprocessing error", {}};
  // Set input status
  diagnostic_handler_->set_input_status(input_status);
  const tam::core::state::types::DiagnosticStatus status =
    diagnostic_handler_->get_diagnostic_status(pose_stamped, tangent_stamped, init_guess_stamped, reg_status);
  EXPECT_EQ(status.level, tam::core::state::types::DiagnosticLevel::ERROR) << "Failed input status check";
  EXPECT_EQ(status.message, "not converged | cloud error | pose valid") << "Failed input message check";
}

/**
 * @brief Test DiagnosticHandler status for a non-converged registration
 */
TEST(DiagnosticHandler, NotConverged)
{
  // clang-format off
  // Construct from config and debug objects
  tam::core::state::types::DiagnosticConfig config;
  tam::core::state::types::DiagnosticDebug debug;
  config.check_input_status = false;
  config.check_forward = false;
  config.check_ellipsis = false;
  config.check_time_diff = false;
  config.enable_vel = true;
  config.check_vel = false;
  std::unique_ptr<tam::core::state::DiagnosticHandler<tam::core::state::types::ICP_EXT>> diagnostic_handler_ = // NOLINT
    tam::core::state::DiagnosticHandler<tam::core::state::types::ICP_EXT>::from_config(config, debug); // NOLINT
  // clang-format on

  // Create pose and initial guess
  const Sophus::SE3f pose =
    Sophus::SE3f(Sophus::SE3f::QuaternionType(1.0, 0.0, 0.0, 0.0), Sophus::SE3f::Point(2.2, 2.0, 3.0));
  const Sophus::SE3f init_guess =
    Sophus::SE3f(Sophus::SE3f::QuaternionType(1.0, 0.0, 0.0, 0.0), Sophus::SE3f::Point(2.0, 2.0, 3.0));
  // Create timestamps
  const std::uint64_t pose_stamp = 19483583754837;
  const std::uint64_t init_guess_stamp = 19483583754837;
  tam::core::state::types::PoseStamped pose_stamped = {pose, pose_stamp};
  tam::core::state::types::TangentStamped tangent_stamped = {Sophus::SE3f::Tangent(), pose_stamp};
  tam::core::state::types::PoseStamped init_guess_stamped = {init_guess, init_guess_stamp};
  // Create registration status and input status object
  tam::core::state::types::RegistrationStatus reg_status{false, 250.0, 450};
  tam::core::state::types::DiagnosticStatus input_status{
    tam::core::state::types::DiagnosticLevel::ERROR, "Preprocessing error", {}};
  // Set input status
  diagnostic_handler_->set_input_status(input_status);
  const tam::core::state::types::DiagnosticStatus status =
    diagnostic_handler_->get_diagnostic_status(pose_stamped, tangent_stamped, init_guess_stamped, reg_status);
  EXPECT_EQ(status.level, tam::core::state::types::DiagnosticLevel::ERROR) << "Failed convergence check";
  EXPECT_EQ(status.message, "not converged | pose valid") << "Failed convergence check";
}

/**
 * @brief Test DiagnosticHandler status for an outdated frame
 */
TEST(DiagnosticHandler, FrameOutdated)
{
  // clang-format off
  // Construct from config and debug objects
  tam::core::state::types::DiagnosticConfig config;
  tam::core::state::types::DiagnosticDebug debug;
  config.check_input_status = false;
  config.check_forward = false;
  config.check_ellipsis = false;
  config.check_time_diff = false;
  config.enable_vel = true;
  config.check_vel = false;
  config.frame_outdated = 100.0;
  std::unique_ptr<tam::core::state::DiagnosticHandler<tam::core::state::types::ICP_EXT>> diagnostic_handler_ = // NOLINT
    tam::core::state::DiagnosticHandler<tam::core::state::types::ICP_EXT>::from_config(config, debug); // NOLINT
  // clang-format on

  // Create pose and initial guess
  const Sophus::SE3f pose =
    Sophus::SE3f(Sophus::SE3f::QuaternionType(1.0, 0.0, 0.0, 0.0), Sophus::SE3f::Point(2.2, 2.0, 3.0));
  const Sophus::SE3f init_guess =
    Sophus::SE3f(Sophus::SE3f::QuaternionType(1.0, 0.0, 0.0, 0.0), Sophus::SE3f::Point(2.0, 2.0, 3.0));
  // Create timestamps
  const std::uint64_t pose_stamp = 19483583754837;
  const std::uint64_t init_guess_stamp = 19483583754837;
  tam::core::state::types::PoseStamped pose_stamped = {pose, pose_stamp};
  tam::core::state::types::TangentStamped tangent_stamped = {Sophus::SE3f::Tangent(), pose_stamp};
  tam::core::state::types::PoseStamped init_guess_stamped = {init_guess, init_guess_stamp};
  // Create registration status and input status object
  tam::core::state::types::RegistrationStatus reg_status{true, 110.0, 200};
  tam::core::state::types::DiagnosticStatus input_status{
    tam::core::state::types::DiagnosticLevel::ERROR, "Preprocessing error", {}};
  // Set input status
  diagnostic_handler_->set_input_status(input_status);
  const tam::core::state::types::DiagnosticStatus status =
    diagnostic_handler_->get_diagnostic_status(pose_stamped, tangent_stamped, init_guess_stamped, reg_status);
  EXPECT_EQ(status.level, tam::core::state::types::DiagnosticLevel::WARN) << "Failed outdated frame check";
  EXPECT_EQ(status.message, "outdated | pose valid") << "Failed oudated frame check";
}

/**
 * @brief Test DiagnosticHandler check for forward movement
 */
TEST(DiagnosticHandler, Forward)
{
  // clang-format off
  // Construct from config and debug objects
  tam::core::state::types::DiagnosticConfig config;
  tam::core::state::types::DiagnosticDebug debug;
  config.check_input_status = false;
  config.check_forward = true;
  config.check_ellipsis = false;
  config.check_time_diff = false;
  config.enable_vel = true;
  config.check_vel = false;
  config.min_motion_threshold = 0.5;
  std::unique_ptr<tam::core::state::DiagnosticHandler<tam::core::state::types::ICP_EXT>> diagnostic_handler_ = // NOLINT
    tam::core::state::DiagnosticHandler<tam::core::state::types::ICP_EXT>::from_config(config, debug); // NOLINT
  // clang-format on

  // Create pose and initial guess
  const Sophus::SE3f pose =
    Sophus::SE3f(Sophus::SE3f::QuaternionType(1.0, 0.0, 0.0, 0.0), Sophus::SE3f::Point(1.0, 2.0, 3.0));
  const Sophus::SE3f init_guess =
    Sophus::SE3f(Sophus::SE3f::QuaternionType(1.0, 0.0, 0.0, 0.0), Sophus::SE3f::Point(2.0, 2.0, 3.0));
  // Create timestamps
  const std::uint64_t pose_stamp = 19483583754837;
  const std::uint64_t init_guess_stamp = 19483583754345;
  tam::core::state::types::PoseStamped pose_stamped = {pose, pose_stamp};
  tam::core::state::types::TangentStamped tangent_stamped = {Sophus::SE3f::Tangent(), pose_stamp};
  tam::core::state::types::PoseStamped init_guess_stamped = {init_guess, init_guess_stamp};
  // Create status objects
  tam::core::state::types::RegistrationStatus reg_status{true, 50.0, 8};
  tam::core::state::types::DiagnosticStatus input_status{
    tam::core::state::types::DiagnosticLevel::ERROR, "Preprocessing error", {}};
  // Set input status
  diagnostic_handler_->set_input_status(input_status);
  // Check pose -> init forward check
  tam::core::state::types::DiagnosticStatus status =
    diagnostic_handler_->get_diagnostic_status(pose_stamped, tangent_stamped, init_guess_stamped, reg_status);
  // Create second pose and initial guess for forward movement evaluation
  const Sophus::SE3f pose2 =
    Sophus::SE3f(Sophus::SE3f::QuaternionType(1.0, 0.0, 0.0, 0.0), Sophus::SE3f::Point(0.9, 2.0, 3.0));
  const Sophus::SE3f init_guess2 =
    Sophus::SE3f(Sophus::SE3f::QuaternionType(1.0, 0.0, 0.0, 0.0), Sophus::SE3f::Point(3.0, 2.0, 3.0));
  tam::core::state::types::PoseStamped pose_stamped2 = {pose2, pose_stamp};
  tam::core::state::types::PoseStamped init_guess_stamped2 = {init_guess2, init_guess_stamp};
  // Check second pose -> forward check possible
  // Create registration status object
  const tam::core::state::types::DiagnosticStatus status2 =
    diagnostic_handler_->get_diagnostic_status(pose_stamped2, tangent_stamped, init_guess_stamped2, reg_status);
  EXPECT_EQ(status2.level, tam::core::state::types::DiagnosticLevel::ERROR) << "Failed Forward check";
  EXPECT_EQ(status2.message, "converged | behind previous") << "Failed Forward check";
}

/**
 * @brief Test DiagnosticHandler check for ellipsis
 */
TEST(DiagnosticHandler, Ellipsis)
{
  // clang-format off
  // Construct from config and debug objects
  tam::core::state::types::DiagnosticConfig config;
  tam::core::state::types::DiagnosticDebug debug;
  config.check_input_status = false;
  config.check_forward = false;
  config.check_ellipsis = true;
  config.check_time_diff = false;
  config.enable_vel = true;
  config.check_vel = false;
  config.ellipsis_size_s = 5.0;
  config.ellipsis_size_d = 3.0;
  std::unique_ptr<tam::core::state::DiagnosticHandler<tam::core::state::types::ICP_EXT>> diagnostic_handler_ = // NOLINT
    tam::core::state::DiagnosticHandler<tam::core::state::types::ICP_EXT>::from_config(config, debug); // NOLINT
  // clang-format on

  // Create pose and initial guess
  const Sophus::SE3f pose =
    Sophus::SE3f(Sophus::SE3f::QuaternionType(1.0, 0.0, 0.0, 0.0), Sophus::SE3f::Point(8.0, 2.0, 3.0));
  const Sophus::SE3f init_guess =
    Sophus::SE3f(Sophus::SE3f::QuaternionType(1.0, 0.0, 0.0, 0.0), Sophus::SE3f::Point(2.0, 2.0, 3.0));
  // Create timestamps
  const std::uint64_t pose_stamp = 19483583754837;
  const std::uint64_t init_guess_stamp = 19483583754345;
  tam::core::state::types::PoseStamped pose_stamped = {pose, pose_stamp};
  tam::core::state::types::TangentStamped tangent_stamped = {Sophus::SE3f::Tangent(), pose_stamp};
  tam::core::state::types::PoseStamped init_guess_stamped = {init_guess, init_guess_stamp};
  // Create registration status and input status object
  tam::core::state::types::RegistrationStatus reg_status{true, 50.0, 6};
  tam::core::state::types::DiagnosticStatus input_status{
    tam::core::state::types::DiagnosticLevel::ERROR, "Preprocessing error", {}};
  // Set input status
  diagnostic_handler_->set_input_status(input_status);
  const tam::core::state::types::DiagnosticStatus status =
    diagnostic_handler_->get_diagnostic_status(pose_stamped, tangent_stamped, init_guess_stamped, reg_status);
  EXPECT_EQ(status.level, tam::core::state::types::DiagnosticLevel::ERROR) << "Failed Ellipsis check";
  EXPECT_EQ(status.message, "converged | too far from guess") << "Failed Ellipsis check";
}

/**
 * @brief Test DiagnosticHandler check for time diff
 */
TEST(DiagnosticHandler, TimeDiff)
{
  // clang-format off
  // Construct from config and debug objects
  tam::core::state::types::DiagnosticConfig config;
  tam::core::state::types::DiagnosticDebug debug;
  config.check_input_status = false;
  config.check_forward = false;
  config.check_ellipsis = false;
  config.check_time_diff = true;
  config.enable_vel = true;
  config.check_vel = false;
  config.max_time_diff = 0.05;
  std::unique_ptr<tam::core::state::DiagnosticHandler<tam::core::state::types::ICP_EXT>> diagnostic_handler_ = // NOLINT
    tam::core::state::DiagnosticHandler<tam::core::state::types::ICP_EXT>::from_config(config, debug); // NOLINT
  // clang-format on

  // Create pose and initial guess
  const Sophus::SE3f pose =
    Sophus::SE3f(Sophus::SE3f::QuaternionType(1.0, 0.0, 0.0, 0.0), Sophus::SE3f::Point(2.0, 4.0, 3.0));
  const Sophus::SE3f init_guess =
    Sophus::SE3f(Sophus::SE3f::QuaternionType(1.0, 0.0, 0.0, 0.0), Sophus::SE3f::Point(2.0, 2.0, 3.0));
  // Create timestamps
  const std::uint64_t pose_stamp = 19483583754837;
  const std::uint64_t init_guess_stamp = 19483683754345;
  tam::core::state::types::PoseStamped pose_stamped = {pose, pose_stamp};
  tam::core::state::types::TangentStamped tangent_stamped = {Sophus::SE3f::Tangent(), pose_stamp};
  tam::core::state::types::PoseStamped init_guess_stamped = {init_guess, init_guess_stamp};
  // Create registration status and input status object
  tam::core::state::types::RegistrationStatus reg_status{true, 50.0, 6};
  tam::core::state::types::DiagnosticStatus input_status{
    tam::core::state::types::DiagnosticLevel::ERROR, "Preprocessing error", {}};
  // Set input status
  diagnostic_handler_->set_input_status(input_status);
  const tam::core::state::types::DiagnosticStatus status =
    diagnostic_handler_->get_diagnostic_status(pose_stamped, tangent_stamped, init_guess_stamped, reg_status);
  EXPECT_EQ(status.level, tam::core::state::types::DiagnosticLevel::ERROR) << "Failed Time Diff check";
  EXPECT_EQ(status.message, "converged | large time diff") << "Failed Time Diff check";
}

/**
 * @brief Test DiagnosticHandler check for forward movement
 */
TEST(DiagnosticHandler, Velocity)
{
  // clang-format off
  // Construct from config and debug objects
  tam::core::state::types::DiagnosticConfig config;
  tam::core::state::types::DiagnosticDebug debug;
  config.check_input_status = false;
  config.check_forward = false;
  config.check_ellipsis = false;
  config.check_time_diff = false;
  config.enable_vel = true;
  config.check_vel = true;
  config.vel_motion_threshold = 2.0;
  std::unique_ptr<tam::core::state::DiagnosticHandler<tam::core::state::types::ICP_EXT>> diagnostic_handler_ = // NOLINT
    tam::core::state::DiagnosticHandler<tam::core::state::types::ICP_EXT>::from_config(config, debug); // NOLINT
  // clang-format on

  // Create pose and initial guess
  const Sophus::SE3f pose =
    Sophus::SE3f(Sophus::SE3f::QuaternionType(1.0, 0.0, 0.0, 0.0), Sophus::SE3f::Point(1.0, 2.0, 3.0));
  const Sophus::SE3f init_guess =
    Sophus::SE3f(Sophus::SE3f::QuaternionType(1.0, 0.0, 0.0, 0.0), Sophus::SE3f::Point(2.0, 2.0, 3.0));
  // Create timestamps
  const std::uint64_t pose_stamp = 19483583754837;
  const std::uint64_t init_guess_stamp = 19483583754345;
  tam::core::state::types::PoseStamped pose_stamped = {pose, pose_stamp};
  tam::core::state::types::TangentStamped tangent_stamped = {Sophus::SE3f::Tangent(), pose_stamp};
  tam::core::state::types::PoseStamped init_guess_stamped = {init_guess, init_guess_stamp};
  // Create status objects
  tam::core::state::types::RegistrationStatus reg_status{true, 50.0, 8};
  tam::core::state::types::DiagnosticStatus input_status{
    tam::core::state::types::DiagnosticLevel::ERROR, "Preprocessing error", {}};
  // Set input status
  diagnostic_handler_->set_input_status(input_status);
  // Check pose -> init forward check
  tam::core::state::types::DiagnosticStatus status =
    diagnostic_handler_->get_diagnostic_status(pose_stamped, tangent_stamped, init_guess_stamped, reg_status);
  // Create second pose and initial guess for forward movement evaluation
  const Sophus::SE3f pose2 =
    Sophus::SE3f(Sophus::SE3f::QuaternionType(1.0, 0.0, 0.0, 0.0), Sophus::SE3f::Point(0.9, 2.0, 3.0));
  const Sophus::SE3f init_guess2 =
    Sophus::SE3f(Sophus::SE3f::QuaternionType(1.0, 0.0, 0.0, 0.0), Sophus::SE3f::Point(3.0, 2.0, 3.0));
  tam::core::state::types::PoseStamped pose_stamped2 = {pose2, pose_stamp};
  tam::core::state::types::PoseStamped init_guess_stamped2 = {init_guess2, init_guess_stamp};
  // Check second pose -> forward check possible
  // Create registration status object
  const tam::core::state::types::DiagnosticStatus status2 =
    diagnostic_handler_->get_diagnostic_status(pose_stamped2, tangent_stamped, init_guess_stamped2, reg_status);
  EXPECT_EQ(status2.level, tam::core::state::types::DiagnosticLevel::WARN) << "Failed Velocity check";
  EXPECT_EQ(status2.message, "converged | below velocity threshold") << "Failed Velocity check";
}

/**
 * @brief Test DiagnosticHandler check that ssa is disabled
 */
TEST(DiagnosticHandler, SSA_Disabled)
{
  // clang-format off
  // Construct from config and debug objects
  tam::core::state::types::DiagnosticConfig config;
  tam::core::state::types::DiagnosticDebug debug;
  config.check_input_status = false;
  config.check_forward = false;
  config.check_ellipsis = false;
  config.check_time_diff = false;
  config.enable_vel = false;
  config.check_vel = true;
  config.vel_motion_threshold = 2.0;
  std::unique_ptr<tam::core::state::DiagnosticHandler<tam::core::state::types::ICP_EXT>> diagnostic_handler_ = // NOLINT
    tam::core::state::DiagnosticHandler<tam::core::state::types::ICP_EXT>::from_config(config, debug); // NOLINT
  // clang-format on

  // Create pose and initial guess
  const Sophus::SE3f pose =
    Sophus::SE3f(Sophus::SE3f::QuaternionType(1.0, 0.0, 0.0, 0.0), Sophus::SE3f::Point(1.0, 2.0, 3.0));
  const Sophus::SE3f init_guess =
    Sophus::SE3f(Sophus::SE3f::QuaternionType(1.0, 0.0, 0.0, 0.0), Sophus::SE3f::Point(2.0, 2.0, 3.0));
  // Create timestamps
  const std::uint64_t pose_stamp = 19483583754837;
  const std::uint64_t init_guess_stamp = 19483583754345;
  tam::core::state::types::PoseStamped pose_stamped = {pose, pose_stamp};
  tam::core::state::types::TangentStamped tangent_stamped = {Sophus::SE3f::Tangent(), pose_stamp};
  tam::core::state::types::PoseStamped init_guess_stamped = {init_guess, init_guess_stamp};
  // Create status objects
  tam::core::state::types::RegistrationStatus reg_status{true, 50.0, 8};
  tam::core::state::types::DiagnosticStatus input_status{
    tam::core::state::types::DiagnosticLevel::ERROR, "Preprocessing error", {}};
  // Set input status
  diagnostic_handler_->set_input_status(input_status);
  // Check pose -> init forward check
  tam::core::state::types::DiagnosticStatus status =
    diagnostic_handler_->get_diagnostic_status(pose_stamped, tangent_stamped, init_guess_stamped, reg_status);
  // Create second pose and initial guess for forward movement evaluation
  const Sophus::SE3f pose2 =
    Sophus::SE3f(Sophus::SE3f::QuaternionType(1.0, 0.0, 0.0, 0.0), Sophus::SE3f::Point(0.9, 2.0, 3.0));
  const Sophus::SE3f init_guess2 =
    Sophus::SE3f(Sophus::SE3f::QuaternionType(1.0, 0.0, 0.0, 0.0), Sophus::SE3f::Point(3.0, 2.0, 3.0));
  tam::core::state::types::PoseStamped pose_stamped2 = {pose2, pose_stamp};
  tam::core::state::types::PoseStamped init_guess_stamped2 = {init_guess2, init_guess_stamp};
  // Check second pose -> forward check possible
  // Create registration status object
  const tam::core::state::types::DiagnosticStatus status2 =
    diagnostic_handler_->get_diagnostic_status(pose_stamped2, tangent_stamped, init_guess_stamped2, reg_status);
  EXPECT_EQ(status2.level, tam::core::state::types::DiagnosticLevel::WARN) << "Failed to detect that vel is disabled";
  EXPECT_EQ(status2.message, "converged | vel disabled") << "Failed to detect that vel is disabled";
}
