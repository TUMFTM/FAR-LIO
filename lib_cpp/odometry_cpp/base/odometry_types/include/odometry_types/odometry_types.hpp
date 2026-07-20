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
#include <map>
#include <memory>
#include <sophus/se3.hpp>
#include <string>
#include <utility>

namespace tam::core::state::types {
/**
 * @brief Linear system for Optimization
 */
struct LinearSystem {
  LinearSystem() = default;

  LinearSystem(const Eigen::Matrix<float, 6, 6>& JTJ_, const Eigen::Matrix<float, 6, 1>& JTr_) : JTJ(JTJ_), JTr(JTr_) {}

  Eigen::Matrix<float, 6, 6> JTJ{Eigen::Matrix<float, 6, 6>::Zero()};
  Eigen::Matrix<float, 6, 1> JTr{Eigen::Matrix<float, 6, 1>::Zero()};
};
/**
 * @brief ENUM for different diagnostic levels
 */
enum class DiagnosticLevel : std::uint8_t { OK = 0, WARN = 1, ERROR = 2, STALE = 3 };

/**
 * @brief Diagnostic status with level, message and key-value pairs
 */
struct DiagnosticStatus {
  DiagnosticLevel level{DiagnosticLevel::STALE};
  std::string message{""};
  std::map<std::string, double> key_values;
};

/**
 * @brief Registration status
 */
struct RegistrationStatus {
  bool converged{false};
  double duration{0.0};
  std::uint64_t num_iter{0};
};

/**
 * @brief Pose with time stamp (internal use for motion model, diagnostics)
 */
struct PoseStamped {
  Sophus::SE3f pose{};
  std::uint64_t stamp{0};
};

/**
 * @brief Tangent (twist) with time stamp (internal use for distortion handling)
 */
struct TangentStamped {
  Sophus::SE3f::Tangent tangent{};
  std::uint64_t stamp{0};
};

/**
 * @brief Pose with covariance (matches geometry_msgs::msg::PoseWithCovariance layout)
 */
struct PoseWithCovariance {
  Sophus::SE3f pose{};
  std::array<float, 36> covariance{};
};

/**
 * @brief Tangent with covariance (matches geometry_msgs::msg::TwistWithCovariance layout)
 */
struct TangentWithCovariance {
  Sophus::SE3f::Tangent tangent{};
  std::array<float, 36> covariance{};
};

/**
 * @brief Odometry output (matches nav_msgs::msg::Odometry layout)
 */
struct Odometry {
  std::uint64_t stamp{0};
  PoseWithCovariance pose{};
  TangentWithCovariance tangent{};
  DiagnosticStatus status{};
};
}  // namespace tam::core::state::types
