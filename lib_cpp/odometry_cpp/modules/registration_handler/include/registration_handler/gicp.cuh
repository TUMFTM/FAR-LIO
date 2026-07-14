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
// strongly base on https://github.com/koide3/small_gicp/tree/master
#pragma once

#include <nvtx3/nvToolsExt.h>
#include <thrust/copy.h>
#include <thrust/device_vector.h>
#include <thrust/execution_policy.h>

#include <algorithm>
#include <iomanip>
#include <iostream>
#include <memory>
#include <sophus/se3.hpp>
#include <string>
#include <utility>
#include <vector>

#include "registration_handler/registration_handler_base.hpp"
#include "robust_kernel/robust_kernel.cuh"
namespace tam::core::state::cuda
{
/**
 * @brief GICP factor
 * @param [in] correspondence Correspondence
 * @param [in] ls_flattened    Flattened linear system
 * @details ls_flattened is a pointer to a flattened linear system, which is
 *          a 6x6 matrix followed by a 6x1 vector. The linear system is built
 *          from the Jacobian and residual of the correspondence.
 * @details The kernel scale and correspondence threshold are set before calling this
 */
template <typename TConfig>
struct GICPFactor : public FactorBase<TConfig>
{
  __device__ void operator()(
    types::Correspondence<TConfig> & correspondence, float * ls_flattened) const override
  {
    if (correspondence.distance < this->correspondence_threshold_) {
      correspondence.precision = Eigen::Matrix3f::Identity();
      if (!correspondence.frame.cov.isZero(1.0e-6f) && !correspondence.map.cov.isZero(1.0e-6f)) {
        const Eigen::Matrix3f mahalanobis =
          (correspondence.map.cov + correspondence.frame.cov.transpose()).inverse();
        // Make sure the covariance is not singular
        correspondence.precision =
          utils::hasNaN(mahalanobis) ? Eigen::Matrix3f::Zero() : mahalanobis;
      }
      Eigen::Vector3f residual = correspondence.frame.pos - correspondence.map.pos;
      float weight = RobustKernelWeight<TConfig>()(residual, this->kernel_scale_);

      Eigen::Matrix<float, 3, 6> J_r;
      J_r.setZero();
      J_r.block<3, 3>(0, 0) = Eigen::Matrix3f::Identity();
      J_r.block<3, 3>(0, 3) = -Sophus::SO3f::hat(correspondence.frame.pos);

      Eigen::Matrix<float, 6, 6> JTJ = J_r.transpose() * weight * correspondence.precision * J_r;
      Eigen::Matrix<float, 6, 1> JTr =
        J_r.transpose() * weight * correspondence.precision * residual;
      // Accumulate the partial sums
      utils::accumulate_ls(JTJ, JTr, ls_flattened);
    }
    return;
  }
};
/**
 * @brief Compute the error of a correspondence for a given transformation
 * @param [in] correspondence Correspondence
 * @param [in] T              Transformation to apply
 * @return                    Error of the correspondence
 * @note:
 * https://github.com/koide3/small_gicp/blob/master/include/small_gicp/factors/gicp_factor.hpp
 */
template <typename TConfig>
struct GICPError : public ErrorBase<TConfig>
{
  __device__ void operator()(
    const types::Correspondence<TConfig> & correspondence, float * sum) const override
  {
    if (correspondence.distance < this->correspondence_threshold_) {
      // Compute residual with double precision
      const Eigen::Vector3f residual = correspondence.map.pos - this->T_ * correspondence.frame.pos;
      // Compute error
      sum[0] += 0.5f * residual.transpose() * correspondence.precision * residual;
    }
    return;
  }
};
template <typename TConfig>
requires types::HASNORMALCOV<TConfig>
class GICP : public RegistrationHandler<TConfig>
{
public:
  /**
   * @brief Constructor for param manager and logger
   */
  static std::unique_ptr<RegistrationHandler<TConfig>> from_config(
    tam::pmg::ParamReferenceManager * pmg, tam::tsl::ReferenceLogger * logger)
  {
    std::unique_ptr<GICP<TConfig>> rh =
      std::unique_ptr<GICP<TConfig>>(new GICP<TConfig>(pmg, logger));
    return rh;
  }
  /**
   * @brief Constructor for config and debug objects
   */
  static std::unique_ptr<RegistrationHandler<TConfig>> from_config(
    const types::RegistrationConfig & config, const types::RegistrationDebug & debug)
  {
    std::unique_ptr<GICP<TConfig>> rh =
      std::unique_ptr<GICP<TConfig>>(new GICP<TConfig>(config, debug));
    return rh;
  }
  /**
   * @brief Register a frame to the map
   * @param [in] frame                    Frame to register
   * @param [in] map                      Map to register to
   * @param [in] initial_guess            Initial guess for the registration
   * @param [in] correspondence_threshold Correspondence threshold
   * @param [in] kernel_scale             Scale of the robust kernel
   * @return                              Transformation from frame to map
   */
  __host__ Sophus::SE3f register_frame(
    const thrust::device_vector<types::Point<TConfig>> & frame, const MapHandler<TConfig> * map,
    const Sophus::SE3f & initial_guess, const float correspondence_threshold,
    const float kernel_scale, ::cuda::stream_ref stream = {}) override
  {
    if (map->empty()) {
      this->debug_.conditional["frame_map_time"] = 0.0;
      return initial_guess;
    }

    // Start timer
    auto start = std::chrono::high_resolution_clock::now();

    // Add points to the local map instance to trigger normal/covariance computation
    thrust::device_vector<types::Point<tam::core::state::types::Point_XYZ>> frame_xyz(frame.size());
    thrust::transform(
      thrust::cuda::par.on(stream.get()), frame.begin(), frame.end(), frame_xyz.begin(),
      [] __device__(const types::Point<TConfig> & src) {
        return tam::core::state::cuda::utils::convert_point<
          TConfig, tam::core::state::types::Point_XYZ>(src);
      });

    this->frame_map_->add_points_device(
      frame_xyz, tam::core::state::types::POINT_NORMAL::MAX_POINTS_PER_VOXEL, 1,
      TConfig::NUM_NEIGHBORS, true, stream);

    // Get back points with normals
    source_ = this->frame_map_->get_cloud();
    utils::transform_points(initial_guess, source_, stream);

    // Compute elapsed time
    // clang-format off
    const double time = std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::high_resolution_clock::now() - start).count() * 1.0e-3;  // NOLINT
    // clang-format on
    this->debug_.conditional["frame_map_time"] = time;

    // Call the solver
    const Sophus::SE3f T = this->solve(
      source_, map, this->gicp_factor_, this->gicp_error_, correspondence_threshold, kernel_scale,
      stream);

    // Spit the final transformation
    return T * initial_guess;
  }
  /**
   * @brief Get correspondences between map and frame for given pose
   * @param [in] points                     Frame with points
   * @param [in] map                        pointer to map handler
   * @param [in] pose                       pose to get correspondences for
   * @param [in] correspondence_threshold   threshold for computation
   * @return vector of correspondences
   */
  __host__ std::vector<types::Correspondence<TConfig>> get_correspondences(
    const thrust::device_vector<types::Point<TConfig>> & points, const MapHandler<TConfig> * map,
    const Sophus::SE3f & pose, const float correspondence_threshold,
    ::cuda::stream_ref stream = {}) const override
  {
    return utils::get_correspondences(
      points, this->correspondences_device_, map, pose, correspondence_threshold, stream);
  }

protected:
  // Inherit constructor from ModelHandler for param manager and logger
  GICP(tam::pmg::ParamReferenceManager * pmg, tam::tsl::ReferenceLogger * logger)
  : RegistrationHandler<TConfig>(pmg, logger)
  {
    // Preallocate vectors on the GPU
    try {
      utils::allocate_vector(source_, 50000);
    } catch (const thrust::system_error & e) {
      std::cerr << "Thrust error: " << e.what() << std::endl;
    }
    // Synchronize to ensure the context is initialized
    cudaDeviceSynchronize();
  }
  // Inherit constructor from ModelHandler for config and debug object
  GICP(const types::RegistrationConfig & config, const types::RegistrationDebug & debug)
  : RegistrationHandler<TConfig>(config, debug)
  {
    // Preallocate vectors on the GPU
    try {
      utils::allocate_vector(source_, 50000);
    } catch (const thrust::system_error & e) {
      std::cerr << "Thrust error: " << e.what() << std::endl;
    }
    // Synchronize to ensure the context is initialized
    cudaDeviceSynchronize();
  }

private:
  // Factor
  GICPFactor<TConfig> gicp_factor_{};
  // Error
  GICPError<TConfig> gicp_error_{};
  // Source Cloud
  thrust::device_vector<types::Point<TConfig>> source_{};
};
}  // namespace tam::core::state::cuda
