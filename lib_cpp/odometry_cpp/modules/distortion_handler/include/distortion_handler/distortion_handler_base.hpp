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
#ifdef __CUDACC__
#include "odometry_cuda_utils/cuda_utils.cuh"
#endif

#include <tbb/blocked_range.h>
#include <tbb/global_control.h>
#include <tbb/parallel_for.h>

#include <chrono>
#include <deque>
#include <eigen3/Eigen/Core>
#include <iostream>
#include <sophus/se3.hpp>
#include <vector>

#include "odometry_base/odometry_base.hpp"
#include "odometry_types/odometry_types.hpp"

namespace tam::core::state {
template <typename TConfig>
class DistortionHandler : public OdometryBase<TConfig, types::DistortionConfig, types::DistortionDebug>
{
public:
  /**
   * @brief Set a pose from an external source (tf)
   */
  void set_pose(const types::PoseStamped& pose)
  {
    // taken from
    // https://github.com/autowarefoundation/autoware_universe/blob/main/sensing/autoware_cuda_pointcloud_preprocessor/src/cuda_pointcloud_preprocessor/cuda_pointcloud_preprocessor_node.cpp
    // Set age of the history to 0.2s
    double pose_history = 0.2;  // seconds
    while (!this->pose_history_.empty()) {
      // rosbag replay
      bool backwards_time_jump_detected = this->pose_history_.front().stamp > pose.stamp;
      // Keep the last 0.2s of history
      bool queue_exceeded = this->pose_history_.front().stamp < pose.stamp - pose_history * 1e9;

      // Clean up pose history
      if (backwards_time_jump_detected) {
        this->pose_history_.clear();
      } else if (queue_exceeded) {
        this->pose_history_.pop_front();
      } else {
        break;
      }
    }

    // Find sorted insert position
    auto it = std::lower_bound(this->pose_history_.begin(), this->pose_history_.end(), pose,
      [](const auto& history, const auto& pose) { return history.stamp < pose.stamp; });
    // Don't add duplicate stamps
    if (it != this->pose_history_.end() && it->stamp == pose.stamp) {
      it->pose = pose.pose;
    } else {
      this->pose_history_.insert(it, pose);
    }
  }

  /**
   * @brief Undistort a frame of points
   * @param [in,out] frame          Frame to undistort
   * @param [in] frame_stamp        Timestamp of the frame (in nanoseconds)
   */
#ifdef __CUDACC__
  virtual __host__ void undistort(thrust::device_vector<types::Point<TConfig>>& frame, const std::uint64_t frame_stamp,
    ::cuda::stream_ref stream = {}) = 0;
#else
  virtual void undistort(std::vector<types::Point<TConfig>>& frame, const std::uint64_t frame_stamp) = 0;
#endif
  /**
   * @brief Initialize threading
   * -> needs to be done after param override
   * @param [in] num_threads        Number of threads to use
   */
  void init(const std::size_t num_threads = 1)
  {
    // This global variable requires static duration storage to be able to manipulate the max
    // concurrency from TBB across the entire class
    static const auto tbb_control_settings =
      tbb::global_control(tbb::global_control::max_allowed_parallelism, static_cast<size_t>(num_threads));
  }

protected:
  // Inherit constructor from OdometryBase for param manager and logger
  DistortionHandler(tam::pmg::ParamReferenceManager* pmg, tam::tsl::ReferenceLogger* logger)
      : OdometryBase<TConfig, types::DistortionConfig, types::DistortionDebug>(pmg, logger)
  {
    this->set_config(pmg);
    this->set_logging(logger);
    // Additional initialization
  }

  // Inherit constructor from OdometryBase for config and debug object
  DistortionHandler(const types::DistortionConfig& config, const types::DistortionDebug& debug)
      : OdometryBase<TConfig, types::DistortionConfig, types::DistortionDebug>(config, debug)
  {
  }

  /**
   * @brief Set the configuration of the distortion handler from the param manager
   * @param [in] pmg                Param manager
   */
  void set_config(tam::pmg::ParamReferenceManager* pmg) override
  {
    // clang-format off
    pmg->declare_parameter("distortion.max_time_diff", &this->config_.max_time_diff, 30.0, tam::pmg::ParameterType::DOUBLE, "Max time diff between pose history and frame stamps (ms)");  // NOLINT
    pmg->declare_parameter("distortion.vel_threshold", &this->config_.vel_threshold, 10.0, tam::pmg::ParameterType::DOUBLE, "Min velocity to apply undistortion (m/s)");  // NOLINT
    // clang-format on
  }

  /**
   * @brief Register the debug variables with the logger
   * @param [in] logger             Logger
   */
  void set_logging(tam::tsl::ReferenceLogger* logger) const override
  {
    logger->log("distortion/valid", &this->debug_.valid);
    logger->log("distortion/undistortion_time", &this->debug_.undistortion_time);
    logger->log("distortion/timestamp_offset", &this->debug_.timestamp_offset);
    logger->log("distortion/num_invalid_points", &this->debug_.num_invalid_points);
  }

  /**
   * @brief Estimate the current linear velocity from the two most recent poses
   * @return velocity magnitude in m/s
   */
  float estimate_velocity() const
  {
    if (this->pose_history_.size() < 2) {
      return 0.0f;
    }
    const auto& prev = this->pose_history_[this->pose_history_.size() - 2];
    const auto& last = this->pose_history_.back();
    const double dt = (static_cast<double>(last.stamp) - static_cast<double>(prev.stamp)) * 1.0e-9;
    if (dt <= 0.0) {
      return 0.0f;
    }
    return (last.pose.translation() - prev.pose.translation()).norm() / static_cast<float>(dt);
  }

  /**
   * @brief Interpolate a pose at the given timestamp from the pose history using SLERP
   * @param [in] stamp              Timestamp to interpolate at (in nanoseconds)
   * @return interpolated pose (extrapolated from the nearest pair if outside the history range)
   * @note assumes pose_history_ holds at least two poses sorted by ascending stamp
   */
  Sophus::SE3f interpolate_pose(const std::uint64_t stamp) const
  {
    const std::size_t n = this->pose_history_.size();
    // Find the first pose with a stamp >= the requested stamp
    auto it = std::lower_bound(this->pose_history_.begin(), this->pose_history_.end(), stamp,
      [](const auto& history, const std::uint64_t s) { return history.stamp < s; });
    // Select the bracketing pair, clamping to the ends for extrapolation
    std::size_t i1 = static_cast<std::size_t>(std::distance(this->pose_history_.begin(), it));
    if (i1 == 0) {
      i1 = 1;
    } else if (i1 >= n) {
      i1 = n - 1;
    }
    const std::size_t i0 = i1 - 1;
    const auto& p0 = this->pose_history_[i0];
    const auto& p1 = this->pose_history_[i1];
    // Interpolation factor (may lie outside [0, 1] when extrapolating)
    const double denom = static_cast<double>(p1.stamp) - static_cast<double>(p0.stamp);
    const float t =
      denom > 0.0 ? static_cast<float>((static_cast<double>(stamp) - static_cast<double>(p0.stamp)) / denom) : 0.0f;
    // Full SE(3) SLERP along the relative twist
    return p0.pose * Sophus::SE3f::exp(t * (p0.pose.inverse() * p1.pose).log());
  }

  /**
   * @brief Fit a ith-order polynomial to the log-poses of the pose history
   * @param [in] frame_stamp        Timestamp of the frame (in nanoseconds)
   * @param [out] poly_coeff        Coefficients of the fitted polynomial (6 x (degree+1))
   * @param [out] timestamp_offset  Offset to add to timestamps
   * @return true if fitting was successful, false otherwise
   * @details The polynomial is fitted to xi_i = log(T_ref^-1 * T_i), where T_ref is the
   *          (SLERP-interpolated) pose at frame_stamp. Evaluating and exponentiating the
   *          polynomial at a point time therefore yields the deskewing correction directly,
   *          expressed relative to the frame reference - T_ref need not be applied per point.
   */
  bool fit_polynom(const std::uint64_t frame_stamp, Eigen::Matrix<float, 6, types::POLYNOM_DEGREE + 1>& poly_coeff,
    float& timestamp_offset)
  {
    // Check if we have enough poses in the range of the frame timestamp
    bool valid_start = false;
    bool valid_end = false;

    // clang-format off
    if (this->pose_history_.size() >= 2) {
      for (size_t i = 0; i < this->pose_history_.size(); ++i) {
        // Check start - assume that we need a pose at least 0.1s before the frame timestamp
        if (!valid_start && std::abs(static_cast<double>(this->pose_history_[i].stamp) - static_cast<double>(frame_stamp - 0.1 * 1e9)) / 1.0e6 < this->config_.max_time_diff) {  // NOLINT
          valid_start = true;
        }
        // Check end
        if (!valid_end && std::abs(static_cast<double>(this->pose_history_[i].stamp) - static_cast<double>(frame_stamp)) / 1.0e6 < this->config_.max_time_diff) {  // NOLINT
          valid_end = true;
        }
      }
    }
    // clang-format on
    if (!valid_start || !valid_end) {
      std::cout << "[DistortionHandler]: Could not interpolate!" << std::endl;
      return false;
    }
    // (Inverse) reference pose at the frame timestamp
    const Sophus::SE3f T_ref_inv = this->interpolate_pose(frame_stamp).inverse();
    // Init variables
    poly_coeff.setZero();
    Eigen::MatrixXf knots(6, pose_history_.size());
    Eigen::VectorXf times(pose_history_.size());

    // Fill matrices
    for (size_t i = 0; i < pose_history_.size(); ++i) {
      // Compute time offset from frame timestamp to poses
      // clang-format off
      times(i) = static_cast<float>((static_cast<double>(pose_history_[i].stamp) - static_cast<double>(frame_stamp)) / 1.0e9);  // NOLINT
      // clang-format on
      // Log-pose relative to the reference frame (se3 twist coordinates)
      knots.col(i) = (T_ref_inv * pose_history_[i].pose).log().matrix();
    }
    // Normalize times
    // Set timestamp offset to earliest pose used for interpolation
    timestamp_offset = -times.minCoeff();
    // Normalize times -> first pose at t=0
    Eigen::VectorXf times_norm = times.array() + timestamp_offset;

    // Fit polynomial (degree = config_.polynom_degree)
    Eigen::MatrixXf A(times_norm.size(), types::POLYNOM_DEGREE + 1);
    for (int i = 0; i < times_norm.size(); ++i) {
      for (int j = 0; j <= types::POLYNOM_DEGREE; ++j) {
        A(i, j) = std::pow(static_cast<float>(times_norm(i)), j);
      }
    }

    // Solve least squares for each twist component and set coefficients
    for (int k = 0; k < 6; ++k) {
      Eigen::VectorXf y = knots.row(k).transpose();
      poly_coeff.row(k) = A.householderQr().solve(y).transpose();
    }
    return true;
  }

protected:
  // Pose history with timestamps
  std::deque<types::PoseStamped> pose_history_{};
};
}  // namespace tam::core::state
