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

#include <atomic>
#include <cmath>
#include <iostream>
#include <memory>
#include <vector>

#include "distortion_handler/distortion_handler_base.hpp"
namespace tam::core::state
{
template <typename TConfig>
class PolynomUndistortion : public DistortionHandler<TConfig>
{
public:
  /**
   * @brief Constructor for param manager and logger
   */
  static std::unique_ptr<DistortionHandler<TConfig>> from_config(
    tam::pmg::ParamReferenceManager * pmg, tam::tsl::ReferenceLogger * logger)
  {
    std::unique_ptr<PolynomUndistortion<TConfig>> dh =
      std::unique_ptr<PolynomUndistortion<TConfig>>(new PolynomUndistortion<TConfig>(pmg, logger));
    return dh;
  }
  /**
   * @brief Constructor for config and debug objects
   */
  static std::unique_ptr<DistortionHandler<TConfig>> from_config(
    const types::DistortionConfig & config, const types::DistortionDebug & debug)
  {
    std::unique_ptr<PolynomUndistortion<TConfig>> ct =
      std::unique_ptr<PolynomUndistortion<TConfig>>(
        new PolynomUndistortion<TConfig>(config, debug));
    return ct;
  }
  /**
   * @brief Undistort a frame of points
   */
  void undistort(
    std::vector<types::Point<TConfig>> & frame, const std::uint64_t frame_stamp) override
  {
    // Return early if no pose history is available or the velocity is below the threshold
    // clang-format off
    if (this->pose_history_.size() < 2 || this->estimate_velocity() < this->config_.vel_threshold) {  // NOLINT
      this->debug_.valid = false;
      this->debug_.undistortion_time = 0.0;
      this->debug_.timestamp_offset = 0.0;
      this->debug_.num_invalid_points = frame.size();
      return;
    }

    auto start = std::chrono::high_resolution_clock::now();
    Eigen::Matrix<float, 6, types::POLYNOM_DEGREE + 1> poly_coeff = Eigen::Matrix<float, 6, types::POLYNOM_DEGREE + 1>::Zero();  // NOLINT
    float timestamp_offset = 0.0;
    const bool valid = this->fit_polynom(frame_stamp, poly_coeff, timestamp_offset);
    // clang-format on
    // Fit polynomial to tangent history
    if (!valid) {
      this->debug_.valid = false;
      this->debug_.undistortion_time = 0.0;
      this->debug_.timestamp_offset = 0.0;
      this->debug_.num_invalid_points = frame.size();
      return;
    }
    int counter = 0;
    // Iterate over frame using TBB parallel_for
    std::atomic<int> atomic_counter{0};
    using points_iterator = typename std::vector<types::Point<TConfig>>::iterator;
    // clang-format off
    const float min_timestamp = static_cast<float>((static_cast<double>(this->pose_history_.front().stamp) - static_cast<double>(frame_stamp)) * 1.0e-9);  // NOLINT
    const float max_timestamp = static_cast<float>((static_cast<double>(this->pose_history_.back().stamp) - static_cast<double>(frame_stamp)) * 1.0e-9);  // NOLINT
    const int degree = types::POLYNOM_DEGREE;
    // clang-format on
    tbb::parallel_for(
      tbb::blocked_range<points_iterator>(frame.begin(), frame.end()),
      [&](const tbb::blocked_range<points_iterator> & r) {
        std::for_each(r.begin(), r.end(), [&](auto & point) {
          // Extract and clamp timestamp to avoid extrapolation.
          float timestamp = std::clamp(point.timestamp, min_timestamp, max_timestamp);

          // Evaluate polynomial using Horner's method -> log-pose relative to the reference frame
          Sophus::SE3f::Tangent twist = Sophus::SE3f::Tangent();
          const float t = timestamp + timestamp_offset;

          for (int k = 0; k < 6; ++k) {
            // Horner's method: evaluate from highest degree down
            float sum = poly_coeff(k, degree);
            for (int j = degree - 1; j >= 0; --j) {
              sum = std::fma(sum, t, poly_coeff(k, j));
            }
            twist(k) = sum;
          }

          // Ensure that the correction is valid (no NaN / Inf)
          if (twist.allFinite()) {
            const auto motion = Sophus::SE3f::exp(twist);
            point.pos = motion * point.pos;
          } else {
            atomic_counter++;
          }
        });
      });
    counter = atomic_counter.load();
    // Compute elapsed time
    // clang-format off
    double unistortion_time = std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::high_resolution_clock::now() - start).count() * 1.0e-3;  // NOLINT
    // clang-format on
    // Update debug information
    this->debug_.valid = valid;
    this->debug_.undistortion_time = unistortion_time;
    this->debug_.timestamp_offset = timestamp_offset;
    this->debug_.num_invalid_points = static_cast<std::int64_t>(counter);
  }

protected:
  // Inherit constructor from ModelHandler for param manager and logger
  PolynomUndistortion(tam::pmg::ParamReferenceManager * pmg, tam::tsl::ReferenceLogger * logger)
  : DistortionHandler<TConfig>(pmg, logger)
  {
  }
  // Inherit constructor from ModelHandler for config and debug object
  PolynomUndistortion(const types::DistortionConfig & config, const types::DistortionDebug & debug)
  : DistortionHandler<TConfig>(config, debug)
  {
  }
};
}  // namespace tam::core::state
