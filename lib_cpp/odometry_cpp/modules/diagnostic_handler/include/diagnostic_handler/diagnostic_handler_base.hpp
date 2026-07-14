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
#include <memory>
#include <sophus/se3.hpp>

#include "odometry_base/odometry_base.hpp"
#include "odometry_types/odometry_types.hpp"
namespace tam::core::state
{
template <typename TConfig>
class DiagnosticHandler
: public OdometryBase<TConfig, types::DiagnosticConfig, types::DiagnosticDebug>
{
public:
  /**
   * @brief Constructor for param manager and logger
   */
  static std::unique_ptr<DiagnosticHandler<TConfig>> from_config(
    tam::pmg::ParamReferenceManager * pmg, tam::tsl::ReferenceLogger * logger)
  {
    std::unique_ptr<DiagnosticHandler<TConfig>> dh =
      std::unique_ptr<DiagnosticHandler<TConfig>>(new DiagnosticHandler<TConfig>(pmg, logger));
    return dh;
  }
  /**
   * @brief Constructor for config and debug objects
   */
  static std::unique_ptr<DiagnosticHandler<TConfig>> from_config(
    const types::DiagnosticConfig & config, const types::DiagnosticDebug & debug)
  {
    std::unique_ptr<DiagnosticHandler<TConfig>> dh =
      std::unique_ptr<DiagnosticHandler<TConfig>>(new DiagnosticHandler<TConfig>(config, debug));
    return dh;
  }
  /**
   * @brief Get the diagnostic status
   * @param [in] pose               Pose to check
   * @param [in] tangent            Tangent (twist) to check
   * @param [in] init_guess         Initial guess
   * @param [in] reg_status         Registration status
   * @return Diagnostic status
   */
  types::DiagnosticStatus get_diagnostic_status(
    const types::PoseStamped & pose, const types::TangentStamped & tangent,
    const types::PoseStamped & init_guess, const types::RegistrationStatus & reg_status)
  {
    types::DiagnosticStatus diag_status;
    // Check registration status
    // Needs to be called first, because it sets the level of the diagnostic status
    // Cannot be deactivated
    this->check_registration(reg_status, diag_status);
    // Check input status
    this->check_input_status(diag_status);
    // Check pose validity
    this->check_pose(pose, tangent, init_guess, diag_status);
    // Update previous pose and initial guess
    if (!this->init_previous_) {
      this->init_previous_ = true;
    }
    this->previous_init_guess_ = init_guess.pose;
    this->previous_pose_ = pose.pose;
    // Set debug values
    this->debug_.overall_status = static_cast<std::int64_t>(diag_status.level);
    return diag_status;
  }
  /**
   * @brief Set the status of the input pointcloud
   * @param [in] status             Status to set
   */
  void set_input_status(const types::DiagnosticStatus & status) { this->input_status_ = status; }

protected:
  // Inherit constructor from OdometryBase for param manager and logger
  DiagnosticHandler(tam::pmg::ParamReferenceManager * pmg, tam::tsl::ReferenceLogger * logger)
  : OdometryBase<TConfig, types::DiagnosticConfig, types::DiagnosticDebug>(pmg, logger)
  {
    this->set_config(pmg);
    this->set_logging(logger);
    // Additional initialization
  }
  // Inherit constructor from OdometryBase for config and debug object
  DiagnosticHandler(const types::DiagnosticConfig & config, const types::DiagnosticDebug & debug)
  : OdometryBase<TConfig, types::DiagnosticConfig, types::DiagnosticDebug>(config, debug)
  {
    // Additional initialization
  }
  /**
   * @brief Set the configuration of the diagnostic handler from the param manager
   * @param [in] pmg                Param manager
   */
  void set_config(tam::pmg::ParamReferenceManager * pmg) override
  {
    // clang-format off
    pmg->declare_parameter("diagnostic.check_input_status", &this->config_.check_input_status, true, tam::pmg::ParameterType::BOOL, "Check status of the input pointcloud");  // NOLINT
    pmg->declare_parameter("diagnostic.frame_outdated", &this->config_.frame_outdated, 100.0, tam::pmg::ParameterType::DOUBLE, "Max registration duration before frame counts as outdated (ms)");  // NOLINT
    pmg->declare_parameter("diagnostic.check_forward", &this->config_.check_forward, true, tam::pmg::ParameterType::BOOL, "Check that the pose moved forward w.r.t. the previous pose");  // NOLINT
    pmg->declare_parameter("diagnostic.min_motion_threshold", &this->config_.min_motion_threshold, 5.0, tam::pmg::ParameterType::DOUBLE, "Min motion between initial guesses to run the forward check (m)");  // NOLINT
    pmg->declare_parameter("diagnostic.check_ellipsis", &this->config_.check_ellipsis, true, tam::pmg::ParameterType::BOOL, "Check that the pose lies within an ellipse around the initial guess");  // NOLINT
    pmg->declare_parameter("diagnostic.ellipsis_size_s", &this->config_.ellipsis_size_s, 5.0, tam::pmg::ParameterType::DOUBLE, "Ellipse semi-axis in longitudinal direction (m)");  // NOLINT
    pmg->declare_parameter("diagnostic.ellipsis_size_d", &this->config_.ellipsis_size_d, 2.5, tam::pmg::ParameterType::DOUBLE, "Ellipse semi-axis in lateral direction (m)");  // NOLINT
    pmg->declare_parameter("diagnostic.check_time_diff", &this->config_.check_time_diff, true, tam::pmg::ParameterType::BOOL, "Check time diff between pose and initial guess");  // NOLINT
    pmg->declare_parameter("diagnostic.max_time_diff", &this->config_.max_time_diff, 50.0, tam::pmg::ParameterType::DOUBLE, "Max time diff between pose and initial guess (ms)");  // NOLINT
    pmg->declare_parameter("diagnostic.enable_vel", &this->config_.enable_vel, false, tam::pmg::ParameterType::BOOL, "Velocity estimation enabled (WARN status otherwise)");  // NOLINT
    pmg->declare_parameter("diagnostic.check_vel", &this->config_.check_vel, true, tam::pmg::ParameterType::BOOL, "Check that the estimated velocity is valid");  // NOLINT
    pmg->declare_parameter("diagnostic.vel_motion_threshold", &this->config_.vel_motion_threshold, 0.25, tam::pmg::ParameterType::DOUBLE, "Min motion / velocity norm for the velocity check"); // NOLINT
    // clang-format on
  }
  /**
   * @brief Register the debug variables with the logger
   * @param [in] logger             Logger
   */
  void set_logging(tam::tsl::ReferenceLogger * logger) const override
  {
    logger->log("diagnostic/overall_status", &this->debug_.overall_status);
    logger->log("diagnostic/registration/status", &this->debug_.registration.status);
    logger->log("diagnostic/input/status", &this->debug_.input.status);
    logger->log("diagnostic/pose/status", &this->debug_.pose.status);
    logger->log("diagnostic/pose/diff_initial_guess_s", &this->debug_.pose.diff_initial_guess_s);
    logger->log("diagnostic/pose/diff_initial_guess_d", &this->debug_.pose.diff_initial_guess_d);
    logger->log("diagnostic/pose/diff_time", &this->debug_.pose.diff_time);
  }

protected:
  /**
   * @brief Set diagnostic status for registration
   * @param [in] reg_status         Registration status
   * @param [out] diag_status       Diagnostic status to update
   */
  void check_registration(
    const types::RegistrationStatus & reg_status, types::DiagnosticStatus & diag_status)
  {
    // Set diagnostic status according to convergence of san registration
    if (!reg_status.converged) {
      diag_status.level = types::DiagnosticLevel::ERROR;
      diag_status.message += "not converged";
    } else if (reg_status.converged && reg_status.duration > this->config_.frame_outdated) {
      diag_status.level = types::DiagnosticLevel::WARN;
      diag_status.message += "outdated";
    } else {
      diag_status.level = types::DiagnosticLevel::OK;
      diag_status.message += "converged";
    }
    // Update debug info
    this->debug_.registration.status = static_cast<std::int64_t>(diag_status.level);
    // Set key value pairs
    diag_status.key_values["registration_time"] = reg_status.duration;
    diag_status.key_values["iterations"] = static_cast<double>(reg_status.num_iter);
  }
  /**
   * @brief Check status of incoming pointcloud
   * @param [out] diag_status       Diagnostic status to update
   */
  void check_input_status(types::DiagnosticStatus & diag_status)
  {
    if (!this->config_.check_input_status) {
      return;
    }
    // Check if input status is ok
    if (this->input_status_.level == types::DiagnosticLevel::OK) {
      // No need to rise the level, because it is already ok
      diag_status.message += " | cloud ok";
      this->debug_.input.status = static_cast<std::int64_t>(types::DiagnosticLevel::OK);
    } else if (this->input_status_.level == types::DiagnosticLevel::WARN) {
      // if (diag_status.level < 1) diag_status.level = 1;
      diag_status.message += " | cloud warning";
      this->debug_.input.status = static_cast<std::int64_t>(types::DiagnosticLevel::WARN);
    } else if (this->input_status_.level == types::DiagnosticLevel::ERROR) {
      diag_status.level = types::DiagnosticLevel::ERROR;
      diag_status.message += " | cloud error";
      this->debug_.input.status = static_cast<std::int64_t>(types::DiagnosticLevel::ERROR);
    } else {
      diag_status.level = types::DiagnosticLevel::ERROR;
      diag_status.message += " | no cloud status";
      this->debug_.input.status = static_cast<std::int64_t>(types::DiagnosticLevel::ERROR);
    }
    return;
  }
  /**
   * @brief Check if the pose is valid
   * @param [in] pose               Pose to check
   * @param [in] tangent            Tangent (twist) to check
   * @param [in] init_guess         Initial guess
   * @param [out] status            Diagnostic status to update
   */
  void check_pose(
    const types::PoseStamped & pose, const types::TangentStamped & tangent,
    const types::PoseStamped & init_guess, types::DiagnosticStatus & status)
  {
    // Check if pose is in front of previous pose
    if (this->config_.check_forward && !forward_check(pose.pose, init_guess.pose, status)) {
      return;
    }
    // Check if pose is within ellipsis around initial guess
    if (this->config_.check_ellipsis && !ellipsis_check(pose.pose, init_guess.pose, status)) {
      return;
    }
    // Check if time difference between pose and initial guess is within bounds
    if (this->config_.check_time_diff && !time_diff_check(pose.stamp, init_guess.stamp, status)) {
      return;
    }
    // Check if velocity is reasonable
    if (this->config_.check_vel && !vel_check(tangent.tangent, init_guess.pose, status)) {
      return;
    }
    // If all checks passed, set status to valid
    status.message += " | pose valid";
    return;
  }
  /**
   * @brief Check if the pose is in front of the previous pose
   * @param [in] pose               Pose to check
   * @param [in] init_guess         Initial guess
   * @param [out] status            Diagnostic status to write the message to
   * @return True if the pose is in front of the previous pose
   */
  bool forward_check(
    const Sophus::SE3f & pose, const Sophus::SE3f & init_guess, types::DiagnosticStatus & status)
  {
    if (this->init_previous_) {
      // Check if vehicle has moved
      if (
        (this->previous_init_guess_.inverse() * init_guess).translation().norm() >
        this->config_.min_motion_threshold) {
        if ((this->previous_pose_.inverse() * pose).translation().x() <= 0.0) {
          status.level = types::DiagnosticLevel::ERROR;
          status.message += " | behind previous";
          this->debug_.pose.status = static_cast<std::int64_t>(types::DiagnosticLevel::ERROR);
          return false;
        }
      }
    }
    this->debug_.pose.status = static_cast<std::int64_t>(types::DiagnosticLevel::OK);
    return true;
  }
  /**
   * @brief Check if the pose is within the ellipsis around the initial guess
   * @param [in] pose               Pose to check
   * @param [in] init_guess         Initial guess
   * @param [out] status            Diagnostic status to write the message to
   * @return True if the pose is within the ellipsis around the initial guess
   */
  bool ellipsis_check(
    const Sophus::SE3f & pose, const Sophus::SE3f & init_guess, types::DiagnosticStatus & status)
  {
    const Sophus::SE3f pose_diff = init_guess.inverse() * pose;
    this->debug_.pose.diff_initial_guess_s = pose_diff.translation().x();
    this->debug_.pose.diff_initial_guess_d = pose_diff.translation().y();
    // Check if point is within ellipsis in s and d
    if (
      pow(pose_diff.translation().x() / this->config_.ellipsis_size_s, 2) +
        pow(pose_diff.translation().y() / this->config_.ellipsis_size_d, 2) <
      1.0) {
      this->debug_.pose.status = static_cast<std::int64_t>(types::DiagnosticLevel::OK);
      return true;
    } else {
      status.level = types::DiagnosticLevel::ERROR;
      status.message += " | too far from guess";
      this->debug_.pose.status = static_cast<std::int64_t>(types::DiagnosticLevel::ERROR);
      return false;
    }
  }
  /**
   * @brief Check if the time difference between pose and initial guess is within bounds
   * @param [in] pose_stamp         Timestamp of the pose
   * @param [in] init_guess_stamp   Timestamp of the initial guess
   * @param [out] status            Diagnostic status to write the message to
   * @return True if the time difference between pose and initial guess is within bounds
   */
  bool time_diff_check(
    const std::uint64_t pose_stamp, const std::uint64_t init_guess_stamp,
    types::DiagnosticStatus & status)
  {
    std::uint64_t time_diff_ns = (pose_stamp > init_guess_stamp) ? (pose_stamp - init_guess_stamp)
                                                                 : (init_guess_stamp - pose_stamp);
    this->debug_.pose.diff_time = time_diff_ns * 1.0e-6;
    if (this->debug_.pose.diff_time < this->config_.max_time_diff) {
      this->debug_.pose.status = static_cast<std::int64_t>(types::DiagnosticLevel::OK);
      return true;
    } else {
      status.level = types::DiagnosticLevel::ERROR;
      status.message += " | large time diff";
      this->debug_.pose.status = static_cast<std::int64_t>(types::DiagnosticLevel::ERROR);
      return false;
    }
  }
  /**
   * @brief Check if the computed velocity is reasonable based on the initial guess
   * @param [in] tangent            Tangent (twist) to check
   * @param [in] init_guess         Initial guess
   * @param [out] status            Diagnostic status to write the message to
   * @return True if the velocity is valid
   */
  bool vel_check(
    const Sophus::SE3f::Tangent & tangent, const Sophus::SE3f & init_guess,
    types::DiagnosticStatus & status)
  {
    // clang-format off
    if (this->init_previous_) {
      // Set the status to warn if the vel is disabled
      if (!this->config_.enable_vel) {
        // Only set status to warn if its not an error already
        if (status.level < types::DiagnosticLevel::WARN) status.level = types::DiagnosticLevel::WARN;  // NOLINT
        status.message += " | vel disabled";
        this->debug_.pose.status = static_cast<std::int64_t>(types::DiagnosticLevel::WARN);
        return false;
      }
      // Check if point is within ellipsis in s and d
      if (
        (this->previous_init_guess_.inverse() * init_guess).translation().norm() <
        this->config_.vel_motion_threshold) {
        // Only set status to warn if its not an error already
        if (status.level < types::DiagnosticLevel::WARN) status.level = types::DiagnosticLevel::WARN;  // NOLINT
        status.message += " | below velocity threshold";
        this->debug_.pose.status = static_cast<std::int64_t>(types::DiagnosticLevel::WARN);
        return false;
      } else {
        // Check if the velocity is valid (i.e. not zeros)
        if (tangent.head<3>().norm() < this->config_.vel_motion_threshold) {
          if (status.level < types::DiagnosticLevel::WARN) status.level = types::DiagnosticLevel::WARN;  // NOLINT
          status.message += " | invalid velocity";
          this->debug_.pose.status = static_cast<std::int64_t>(types::DiagnosticLevel::WARN);
          return false;
        }
      }
    }
    this->debug_.pose.status = static_cast<std::int64_t>(types::DiagnosticLevel::OK);
    return true;
    // clang-format on
  }
  // Member variables
  bool init_previous_{false};
  types::DiagnosticStatus input_status_{};
  Sophus::SE3f previous_pose_;
  Sophus::SE3f previous_init_guess_;
};
}  // namespace tam::core::state
