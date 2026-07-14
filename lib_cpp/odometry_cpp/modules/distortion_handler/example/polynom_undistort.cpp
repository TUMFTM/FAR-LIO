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
#include <vector>

#include "distortion_handler/polynom_undistortion.hpp"
#include "odometry_utils/utils.hpp"
#include "param_management_cpp/param_reference_manager.hpp"
#include "tsl_logger_cpp/reference_logger.hpp"
int main()
{
  // clang-format off
  // Construct from param manager and logger
  tam::pmg::ParamReferenceManager::UniquePtr pmg_ =  std::make_unique<tam::pmg::ParamReferenceManager>(); // NOLINT
  tam::tsl::ReferenceLogger::UniquePtr logger_ = std::make_unique<tam::tsl::ReferenceLogger>(); // NOLINT
  std::unique_ptr<tam::core::state::DistortionHandler<tam::core::state::types::ICP_EXT>> undistort_ =  // NOLINT
    tam::core::state::PolynomUndistortion<tam::core::state::types::ICP_EXT>::from_config(pmg_.get(), logger_.get()); // NOLINT
  // clang-format on

  tam::pmg::MgmtInterface * pmg_raw = pmg_.get();
  (void)pmg_raw;

  // Initialize a pose history by integrating accelerating velocities
  std::vector<tam::core::state::types::PoseStamped> poses{};
  // Capture current unix timestamp
  std::uint64_t current_time = std::chrono::duration_cast<std::chrono::nanoseconds>(
                                 std::chrono::system_clock::now().time_since_epoch())
                                 .count();

  // Random distribution for noise
  std::random_device rd;
  std::mt19937 gen(rd());
  std::normal_distribution<> dis(0.0, 1.0);
  Eigen::Vector3f velocity(20.0f, 0.5f, 0.0f);
  Eigen::Vector3f acceleration(3.0f, 0.5f, 0.0f);
  constexpr float dt = 0.01f;  // 10ms between poses
  // Accumulated pose, integrated from the velocity profile
  Sophus::SE3f pose{};

  for (size_t i = 0; i < 20; ++i) {
    tam::core::state::types::PoseStamped pose_stamped{};
    pose_stamped.stamp = current_time + i * 1e7;  // 10ms apart
    // Define velocities
    velocity = Eigen::Vector3f(
      velocity.x() + acceleration.x() * dt + dis(gen),
      velocity.y() + acceleration.y() * dt + 0.1f * dis(gen),
      velocity.z() + acceleration.z() * dt + 0.1f * dis(gen));
    // Integrate velocity into the accumulated pose (small yaw rate to exercise rotation)
    Sophus::SE3f::Tangent twist;
    twist << velocity * dt, 0.0f, 0.0f, 0.1f * dt;
    pose = pose * Sophus::SE3f::exp(twist);
    pose_stamped.pose = pose;
    std::cout << "Pose at time " << pose_stamped.stamp << ": "
              << pose.translation().transpose() << std::endl;
    poses.push_back(pose_stamped);
    undistort_->set_pose(pose_stamped);
  }

  // Trigger undistortion with frame time shortly after time of last pose
  std::vector<tam::core::state::types::Point<tam::core::state::types::ICP_EXT>> frame{};
  std::uint64_t frame_stamp = poses.back().stamp + 1e7;
  std::cout << "Undistorting frame at time " << frame_stamp << std::endl;

  // Generate vector of points around the origin with noise
  // Points generated circular around origin with stamps decreasing from frame_stamp
  for (size_t i = 0; i < 10; ++i) {
    tam::core::state::types::Point<tam::core::state::types::ICP_EXT> point{};
    point.pos = Eigen::Vector3f(
      5.0f * std::cos(i * 2.0f * std::numbers::pi_v<float> / 10.0f) + dis(gen),
      5.0f * std::sin(i * 2.0f * std::numbers::pi_v<float> / 10.0f) + dis(gen), 0.0f);
    point.timestamp = -1e-3 * i;
    frame.push_back(point);
  }

  std::cout << "Original points:" << std::endl;
  for (size_t i = 0; i < frame.size(); ++i)
    std::cout << "Index: " << i << ", Point: " << frame[i].pos.transpose() << std::endl;

  undistort_->undistort(frame, frame_stamp);

  std::cout << "Undistorted points:" << std::endl;
  for (size_t i = 0; i < frame.size(); ++i) {
    std::cout << "Index: " << i
              << ", Timestamp: " << frame[i].timestamp
              << ", Point: " << frame[i].pos.transpose() << std::endl;
  }

  // clang-format off
  std::cout << "Undistortion valid: " << undistort_->get_debug().valid << std::endl;
  std::cout << "Number of invalid points: " << undistort_->get_debug().num_invalid_points << std::endl;  // NOLINT
  std::cout << "Undistortion time: " << undistort_->get_debug().undistortion_time << " ms" << std::endl;  // NOLINT
  // clang-format on

  // Plot results with rerun
#ifdef USE_VISUALIZATION

  auto rec = tam::core::state::utils::spawn_rerun_stream("undistortion");

  // Knots
  rec.log_static(
    "undistort/x_knots", rerun::SeriesLines()
                           .with_colors(rerun::Rgba32{255, 0, 0})
                           .with_names("x_knots")
                           .with_widths(4.0f));
  rec.log_static(
    "undistort/y_knots", rerun::SeriesLines()
                           .with_colors(rerun::Rgba32{0, 255, 0})
                           .with_names("y_knots")
                           .with_widths(4.0f));

  // Log the data on a timeline called "step".
  for (size_t i = 0; i < poses.size(); ++i) {
    rec.set_time_sequence("step", poses[i].stamp);

    rec.log("undistort/x_knots", rerun::Scalars(poses[i].pose.translation().x()));
    rec.log("undistort/y_knots", rerun::Scalars(poses[i].pose.translation().y()));
  }

  // Fitted polynomial
  rec.log_static(
    "undistort/x",
    rerun::SeriesLines().with_colors(rerun::Rgba32{255, 0, 0}).with_names("x").with_widths(1.0f));
  rec.log_static(
    "undistort/y",
    rerun::SeriesLines().with_colors(rerun::Rgba32{0, 255, 0}).with_names("y").with_widths(1.0f));

  // TODO(Maximilian): Log fitted polynomial for visualization
#endif
  return 0;
}
