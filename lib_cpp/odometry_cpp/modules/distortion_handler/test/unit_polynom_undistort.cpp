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

#include "distortion_handler/polynom_undistortion.hpp"
#include "param_management_cpp/param_reference_manager.hpp"
#include "tsl_logger_cpp/reference_logger.hpp"

/**
 * @brief Test construction of PolynomUndistortion from param manager and logger
 */
TEST(PolynomUndistortion, BuildPmgLogger)
{
  // clang-format off
  // Construct from param manager and logger
  tam::pmg::ParamReferenceManager::UniquePtr pmg_ =  std::make_unique<tam::pmg::ParamReferenceManager>(); // NOLINT
  tam::tsl::ReferenceLogger::UniquePtr logger_ = std::make_unique<tam::tsl::ReferenceLogger>(); // NOLINT
  std::unique_ptr<tam::core::state::DistortionHandler<tam::core::state::types::ICP_EXT>> undistort_ =  // NOLINT
    tam::core::state::PolynomUndistortion<tam::core::state::types::ICP_EXT>::from_config(pmg_.get(), logger_.get()); // NOLINT
  // clang-format on

  tam::pmg::MgmtInterface* pmg_raw = pmg_.get();
  (void)pmg_raw;

  EXPECT_EQ(undistort_->get_debug().undistortion_time, 0.0)
    << "Failed to construct PolynomUndistortion from param manager and logger";
}

/**
 * @brief Test construction of PolynomUndistortion from config and debug object
 */
TEST(PolynomUndistortion, BuildConfigDebug)
{
  // clang-format off
  // Construct from config and debug objects
  tam::core::state::types::DistortionConfig config;
  tam::core::state::types::DistortionDebug debug;
  std::unique_ptr<tam::core::state::DistortionHandler<tam::core::state::types::ICP_EXT>> undistort_ =  // NOLINT
    tam::core::state::PolynomUndistortion<tam::core::state::types::ICP_EXT>::from_config(config, debug);  // NOLINT
  // clang-format on

  EXPECT_EQ(undistort_->get_debug().undistortion_time, 0.0)
    << "Failed to construct PolynomUndistortion from param config and debug";
}

/**
 * @brief Test undistortion of a frame with known poses and timestamps
 */
TEST(PolynomUndistortion, UndistortFrame)
{
  // clang-format off
  // Construct from param manager and logger
  tam::pmg::ParamReferenceManager::UniquePtr pmg_ =  std::make_unique<tam::pmg::ParamReferenceManager>(); // NOLINT
  tam::tsl::ReferenceLogger::UniquePtr logger_ = std::make_unique<tam::tsl::ReferenceLogger>(); // NOLINT
  std::unique_ptr<tam::core::state::DistortionHandler<tam::core::state::types::ICP_EXT>> undistort_ =  // NOLINT
    tam::core::state::PolynomUndistortion<tam::core::state::types::ICP_EXT>::from_config(pmg_.get(), logger_.get()); // NOLINT
  // clang-format on

  // Initialize a pose history by integrating accelerating velocities (translation only, so the
  // x-monotonicity assertion below holds)
  std::vector<tam::core::state::types::PoseStamped> poses{};
  // Capture current unix timestamp
  std::uint64_t current_time =
    std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::system_clock::now().time_since_epoch()).count();

  // Random distribution for noise
  std::random_device rd;
  std::mt19937 gen(rd());
  std::normal_distribution<> dis(0.0, 1.0);
  Eigen::Vector3f velocity(20.0f, 0.5f, 0.0f);
  Eigen::Vector3f acceleration(3.0f, 0.5f, 0.0f);
  constexpr float dt = 0.01f;  // 10ms between poses
  Eigen::Vector3f position(0.0f, 0.0f, 0.0f);

  for (size_t i = 0; i < 20; ++i) {
    tam::core::state::types::PoseStamped pose_stamped{};
    pose_stamped.stamp = current_time + i * 1e7;  // 10ms apart
    // Define velocities
    velocity = Eigen::Vector3f(velocity.x() + acceleration.x() * dt + dis(gen),
      velocity.y() + acceleration.y() * dt + 0.1f * dis(gen), velocity.z() + acceleration.z() * dt + 0.1f * dis(gen));
    // Integrate velocity into the position (identity rotation)
    position += velocity * dt;
    pose_stamped.pose = Sophus::SE3f(Eigen::Quaternionf::Identity(), position);
    poses.push_back(pose_stamped);
    undistort_->set_pose(pose_stamped);
  }

  // Trigger undistortion with frame time shortly after time of last pose
  std::vector<tam::core::state::types::Point<tam::core::state::types::ICP_EXT>> frame{};
  std::uint64_t frame_stamp = poses.back().stamp + 1e7;

  // Generate vector of points around the origin with noise
  // Points generated circular around origin with stamps decreasing from frame_stamp
  for (size_t i = 0; i < 10; ++i) {
    tam::core::state::types::Point<tam::core::state::types::ICP_EXT> point{};
    point.pos = Eigen::Vector3f(5.0f * std::cos(i * 2.0f * std::numbers::pi_v<float> / 10.0f) + dis(gen),
      5.0f * std::sin(i * 2.0f * std::numbers::pi_v<float> / 10.0f) + dis(gen), 0.0f);
    point.timestamp = -1e-3 * i;
    frame.push_back(point);
  }
  // clang-format off
  // Capture a copy of the frame for comparison after undistortion
  std::vector<tam::core::state::types::Point<tam::core::state::types::ICP_EXT>> frame_original = frame;  // NOLINT
  // Undistort the frame
  undistort_->undistort(frame, frame_stamp);
  // Check that undistortion was successful
  EXPECT_TRUE(undistort_->get_debug().valid) << "Undistortion failed due to invalid polynomial fitting";  // NOLINT
  EXPECT_EQ(undistort_->get_debug().num_invalid_points, 0) << "Undistortion failed due to invalid points";  // NOLINT
  // clang-format on
  // Following the timestamp convention, the points x-coordinates should be lower than before,
  // as the motion is in positive x direction and points have negative timestamps
  for (size_t i = 0; i < frame.size(); ++i) {
    EXPECT_LE(frame[i].pos.x(), frame_original[i].pos.x())
      << "Undistorted point x-coordinate is not equal or smaller than original:" << frame[i].pos.x()
      << " >= " << frame_original[i].pos.x();
  }
}
