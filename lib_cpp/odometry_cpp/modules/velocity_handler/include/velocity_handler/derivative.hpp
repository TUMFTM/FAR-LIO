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

#include <cmath>

#include <chrono>
#include <eigen3/Eigen/Core>
#include <iostream>
#include <memory>
#include <optional>
#include <sophus/se3.hpp>
#include <vector>

#include "velocity_handler/velocity_handler_base.hpp"
namespace tam::core::state
{
/**
 * @brief Pose-derivative velocity handler.
 */
template <typename TConfig>
class Derivative : public VelocityHandler<TConfig>
{
public:
  /**
   * @brief Constructor for param manager and logger
   */
  static std::unique_ptr<VelocityHandler<TConfig>> from_config(
    tam::pmg::ParamReferenceManager * pmg, tam::tsl::ReferenceLogger * logger)
  {
    std::unique_ptr<Derivative<TConfig>> vh =
      std::unique_ptr<Derivative<TConfig>>(new Derivative<TConfig>(pmg, logger));
    return vh;
  }
  /**
   * @brief Constructor for config and debug objects
   */
  static std::unique_ptr<VelocityHandler<TConfig>> from_config(
    const types::VelocityConfig & config, const types::VelocityDebug & debug)
  {
    std::unique_ptr<Derivative<TConfig>> vh =
      std::unique_ptr<Derivative<TConfig>>(new Derivative<TConfig>(config, debug));
    return vh;
  }
  /**
   * @brief Estimate twist from pose pair. The frame argument is unused.
   * @param[in] frame                   The input frame of points (unused).
   * @param[in] pose_registered         The current registered pose
   */
  types::TangentStamped get_tangent(
    [[maybe_unused]] const std::vector<types::Point<TConfig>> & frame,
    const types::PoseStamped & pose_registered) override
  {
    const auto t_start = std::chrono::high_resolution_clock::now();
    types::TangentStamped result{};
    result.stamp = pose_registered.stamp;

    // Return if the buffer is empty or the handler is not initialized
    if (!previous_pose_.has_value()) {
      std::cerr << "[VelocityHandler::Derivative]: Velocity estimation not initialized!"
                << std::endl;
      previous_pose_ = pose_registered;
      result.tangent << Eigen::Matrix<float, 6, 1>::Zero();
      finalize(t_start, 0.0);
      return result;
    }

    // Compute difference in timestamps to the last buffered pose
    const double dt = (pose_registered.stamp - previous_pose_.value().stamp) * 1e-9;

    // Check for invalid time delta
    if (!std::isfinite(dt) || std::abs(dt) < 1e-6) {
      std::cerr << "[VelocityHandler::Derivative]: Invalid time delta: " << dt << std::endl;
      // Keep previous velocity or set zero
      previous_pose_ = pose_registered;
      result.tangent << Eigen::Matrix<float, 6, 1>::Zero();
      finalize(t_start, dt);
      return result;
    }
    // Check if dt is too large
    if (std::abs(dt) > 1.0f) {
      std::cerr << "[VelocityHandler::Derivative]: Time delta too large: " << dt
                << "s. Resetting velocity." << std::endl;
      previous_pose_ = pose_registered;
      result.tangent << Eigen::Matrix<float, 6, 1>::Zero();
      finalize(t_start, dt);
      return result;
    }

    // Compute velocity using the logarithmic map of the SE3 difference
    // ==== Extract of the sophus docs ====
    // For the derivation of the logarithm of SE(3), see
    // J. Gallier, D. Xu, "Computing exponentials of skew symmetric matrices
    // and logarithms of orthogonal matrices", IJRA 2002.
    // https:///pdfs.semanticscholar.org/cfe3/e4b39de63c8cabd89bf3feff7f5449fc981d.pdf
    // (Sec. 6., pp. 8)
    // ====================================
    const Sophus::SE3f delta = previous_pose_.value().pose.inverse() * pose_registered.pose;
    result.tangent = delta.log() / static_cast<float>(dt);

    // Validate linear velocity (check for NaNs and unreasonably high speeds)
    if (!result.tangent.head<3>().allFinite() || result.tangent.head<3>().norm() > 100.0f) {
      std::cerr << "[VelocityHandler::Derivative]: Invalid linear velocity computed: "
                << result.tangent.head<3>().transpose() << std::endl;
      result.tangent.head<3>() = Eigen::Vector3f::Zero();
    }
    // Validate angular velocity (check for NaNs and unreasonably high speeds)
    if (!result.tangent.tail<3>().allFinite() || result.tangent.tail<3>().norm() > 10.0f) {
      std::cerr << "[VelocityHandler::Derivative]: Invalid angular velocity computed: "
                << result.tangent.tail<3>().transpose() << std::endl;
      result.tangent.tail<3>() = Eigen::Vector3f::Zero();
    }
    // Buffer the current pose for next iteration
    previous_pose_ = pose_registered;
    finalize(t_start, dt);
    return result;
  }

protected:
  Derivative(tam::pmg::ParamReferenceManager * pmg, tam::tsl::ReferenceLogger * logger)
  : VelocityHandler<TConfig>(pmg, logger)
  {
  }
  Derivative(const types::VelocityConfig & config, const types::VelocityDebug & debug)
  : VelocityHandler<TConfig>(config, debug)
  {
  }

private:
  void finalize(const std::chrono::high_resolution_clock::time_point t_start, const double vel_dt)
  {
    // clang-format off
    this->debug_.velocity_time = std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::high_resolution_clock::now() - t_start).count() * 1.0e-3;  // NOLINT
    this->debug_.conditional["vel_dt"] = vel_dt;
    // clang-format on
  }

protected:
  std::optional<types::PoseStamped> previous_pose_{};
};
}  // namespace tam::core::state
