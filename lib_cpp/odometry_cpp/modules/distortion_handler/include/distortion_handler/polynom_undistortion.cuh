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

#include <cstdio>
#include <iostream>
#include <memory>

#include "distortion_handler/distortion_handler_base.hpp"
namespace tam::core::state::cuda
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
    std::unique_ptr<PolynomUndistortion<TConfig>> dh =
      std::unique_ptr<PolynomUndistortion<TConfig>>(
        new PolynomUndistortion<TConfig>(config, debug));
    return dh;
  }
  /**
   * @brief Undistort a frame of points
   */
  __host__ void undistort(
    thrust::device_vector<types::Point<TConfig>> & frame, const std::uint64_t frame_stamp,
    ::cuda::stream_ref stream = {}) override
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
    nvtxRangePush("undistort");
    // Start timer
    auto start = std::chrono::high_resolution_clock::now();

    Eigen::Matrix<float, 6, types::POLYNOM_DEGREE + 1> poly_coeff = Eigen::Matrix<float, 6, types::POLYNOM_DEGREE + 1>::Zero();  // NOLINT
    float timestamp_offset = 0.0;
    const bool valid = this->fit_polynom(frame_stamp, poly_coeff, timestamp_offset);
    // clang-format on
    // Fit polynomial to pose history
    if (!valid) {
      nvtxRangePop();
      // Update debug information
      this->debug_.valid = valid;
      this->debug_.undistortion_time = 0.0;
      this->debug_.timestamp_offset = 0.0;
      this->debug_.num_invalid_points = frame.size();
      return;
    }

    // Copy variables to capture for device copy
    const double min_pose_time =
      static_cast<double>(this->pose_history_.front().stamp) * 1.0e-9;
    const double max_pose_time =
      static_cast<double>(this->pose_history_.back().stamp) * 1.0e-9;
    const double frame_stamp_s = static_cast<double>(frame_stamp) * 1.0e-9;
    const float max_diff = static_cast<float>(min_pose_time - frame_stamp_s);
    const float min_diff = static_cast<float>(max_pose_time - frame_stamp_s);
    int degree = types::POLYNOM_DEGREE;
    // Copy coefficients to device
    Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor> poly_coeff_row_major =
      poly_coeff;
    float * raw_pointer_poly_coeff = thrust::raw_pointer_cast(this->poly_coeff_d_.data());
    cudaMemcpy(
      raw_pointer_poly_coeff, poly_coeff_row_major.data(),
      6 * (types::POLYNOM_DEGREE + 1) * sizeof(float), cudaMemcpyHostToDevice);

    // Setup counter
    unsigned int h_counter = 0;
    cudaMemset(this->counter_d_, 0, sizeof(unsigned int));
    unsigned int * counter_ptr = this->counter_d_;

    // Iterate over frame
    thrust::for_each(
      thrust::cuda::par.on(stream.get()), frame.begin(), frame.end(),
      [min_diff, max_diff, degree, raw_pointer_poly_coeff, timestamp_offset,
       counter_ptr] __device__(auto & point) {
        // Restrict pointer access to coefficients
        const float * __restrict__ coeff = raw_pointer_poly_coeff;
        // Extract and clamp timestamp to avoid extrapolation
        float timestamp = point.timestamp;
        timestamp = fmaxf(timestamp, max_diff);
        timestamp = fminf(timestamp, min_diff);

        // Evaluate polynomial using Horner's method (much faster than pow on device)
        // -> log-pose relative to the reference frame
        Sophus::SE3f::Tangent twist = Sophus::SE3f::Tangent();
        const float t = timestamp + timestamp_offset;

#pragma unroll
        for (int k = 0; k < 6; ++k) {
          // Capture base index for better memory access pattern
          const int base_ind = k * (degree + 1);
          // Horner's method: evaluate from highest degree down
          float sum = coeff[base_ind + degree];
#pragma unroll
          for (int j = degree - 1; j >= 0; --j) {
            sum = fmaf(sum, t, coeff[base_ind + j]);
          }
          twist(k) = sum;
        }

        // Ensure that the correction is valid (no NaN / Inf)
        if (!utils::hasNaN<6, 1>(twist) && isfinite(twist.squaredNorm())) {
          const auto motion = Sophus::SE3f::exp(twist);
          point.pos = motion * point.pos;
        } else {
          atomicAdd(counter_ptr, 1);
        }
      });
    // Wait for synchronization
    stream.wait();
    cudaMemcpy(&h_counter, this->counter_d_, sizeof(unsigned int), cudaMemcpyDeviceToHost);
    // Compute elapsed time
    // clang-format off
    double unistortion_time = std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::high_resolution_clock::now() - start).count() * 1.0e-3;  // NOLINT
    // clang-format on
    // Update debug information
    this->debug_.valid = valid;
    this->debug_.undistortion_time = unistortion_time;
    this->debug_.timestamp_offset = timestamp_offset;
    this->debug_.num_invalid_points = static_cast<std::int64_t>(h_counter);
    nvtxRangePop();
  }

protected:
  // Inherit constructor from ModelHandler for param manager and logger
  PolynomUndistortion(tam::pmg::ParamReferenceManager * pmg, tam::tsl::ReferenceLogger * logger)
  : DistortionHandler<TConfig>(pmg, logger)
  {
    // Allocate the counter on the device
    cudaMalloc(&counter_d_, sizeof(unsigned int));
    cudaMemset(counter_d_, 0, sizeof(unsigned int));
    // Allocate the polynomial coefficients on the device
    utils::allocate_vector(this->poly_coeff_d_, 6 * (types::POLYNOM_DEGREE + 1));
  }
  // Inherit constructor from ModelHandler for config and debug object
  PolynomUndistortion(const types::DistortionConfig & config, const types::DistortionDebug & debug)
  : DistortionHandler<TConfig>(config, debug)
  {
    // Allocate the counter on the device
    cudaMalloc(&counter_d_, sizeof(unsigned int));
    cudaMemset(counter_d_, 0, sizeof(unsigned int));
    utils::allocate_vector(this->poly_coeff_d_, 6 * (types::POLYNOM_DEGREE + 1));
  }

private:
  mutable unsigned int * counter_d_;
  mutable thrust::device_vector<float> poly_coeff_d_;
};
}  // namespace tam::core::state::cuda
