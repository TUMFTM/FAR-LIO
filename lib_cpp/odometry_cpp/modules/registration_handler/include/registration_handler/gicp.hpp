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

#include <algorithm>
#include <iomanip>
#include <iostream>
#include <memory>
#include <sophus/se3.hpp>
#include <utility>
#include <vector>

#include "registration_handler/registration_handler_base.hpp"
namespace tam::core::state
{
/**
 * @brief Build the linear system for the GICP algorithm
 * @param [in] correspondence         Correspondence
 * @param [in] kernel_scale            Scale of the robust kernel
 * @return                             Linear system
 * @note                               This is a functor to be used with the parallel
 *                                     reduction in the build_linear_system function
 */
template <typename TConfig>
struct GICPFactor : public FactorBase<TConfig>
{
  types::LinearSystem operator()(types::Correspondence<TConfig> & correspondence) const override
  {
    // Compute precision matrix from correspondence covs
    correspondence.precision = Eigen::Matrix3f::Identity();
    if (!correspondence.frame.cov.isZero(1.0e-6f) && !correspondence.map.cov.isZero(1.0e-6f)) {
      const Eigen::Matrix3f mahalanobis =
        (correspondence.map.cov + correspondence.frame.cov).inverse();
      // Make sure the covariance is not singular
      correspondence.precision = (mahalanobis.hasNaN()) ? Eigen::Matrix3f::Zero() : mahalanobis;
    }
    const Eigen::Vector3f residual = correspondence.frame.pos - correspondence.map.pos;
    Eigen::Matrix<float, 3, 6> J_r;
    J_r.block<3, 3>(0, 0) = Eigen::Matrix3f::Identity();
    J_r.block<3, 3>(0, 3) = -1.0 * Sophus::SO3f::hat(correspondence.frame.pos);
    // Compute kernel weight
    const float w = robust_kernel_weight<TConfig>(residual, this->kernel_scale_);
    return types::LinearSystem(
      J_r.transpose() * w * correspondence.precision * J_r,        // JTJ
      J_r.transpose() * w * correspondence.precision * residual);  // JTr
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
  float operator()(const types::Correspondence<TConfig> & correspondence) const override
  {
    // Compute residual with double precision
    const Eigen::Vector3f residual = correspondence.map.pos - this->T_ * correspondence.frame.pos;
    // Compute error
    return 0.5f * residual.transpose() * correspondence.precision * residual;
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
  Sophus::SE3f register_frame(
    const std::vector<types::Point<TConfig>> & frame, const MapHandler<TConfig> * map,
    const Sophus::SE3f & initial_guess, const float correspondence_threshold,
    const float kernel_scale) override
  {
    if (map->empty()) {
      this->debug_.conditional["frame_map_time"] = 0.0;
      return initial_guess;
    }
    // Start timer
    auto start = std::chrono::high_resolution_clock::now();

    // Add points to the local map instance to trigger normal/covariance computation
    std::vector<types::Point<tam::core::state::types::Point_XYZ>> point_xyz;
    point_xyz.reserve(frame.size());
    std::transform(frame.begin(), frame.end(), std::back_inserter(point_xyz), [](const auto & pt) {
      return tam::core::state::utils::convert_point<TConfig, tam::core::state::types::Point_XYZ>(
        pt);
    });
    this->frame_map_->add_points(point_xyz);

    // Get back points with normals
    std::vector<types::Point<TConfig>> source = this->frame_map_->get_cloud();
    utils::transform_points(initial_guess, source);

    // Compute elapsed time
    // clang-format off
    const double time = std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::high_resolution_clock::now() - start).count() * 1.0e-3;  // NOLINT
    // clang-format on
    this->debug_.conditional["frame_map_time"] = time;

    // Call the solver
    const Sophus::SE3f T = this->solve(
      source, map, this->gicp_factor_, this->gicp_error_, correspondence_threshold, kernel_scale);

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
  std::vector<types::Correspondence<TConfig>> get_correspondences(
    const std::vector<types::Point<TConfig>> & points, const MapHandler<TConfig> * map,
    const Sophus::SE3f & pose, const float correspondence_threshold) const override
  {
    std::vector<types::Point<TConfig>> frame = points;
    utils::transform_points(pose, frame);
    return utils::get_correspondences(frame, map, correspondence_threshold);
  }

protected:
  // Inherit constructor from ModelHandler for param manager and logger
  GICP(tam::pmg::ParamReferenceManager * pmg, tam::tsl::ReferenceLogger * logger)
  : RegistrationHandler<TConfig>(pmg, logger)
  {
    // Additional initialization
  }
  // Inherit constructor from ModelHandler for config and debug object
  GICP(const types::RegistrationConfig & config, const types::RegistrationDebug & debug)
  : RegistrationHandler<TConfig>(config, debug)
  {
    // Additional initialization
  }

private:
  // Factor
  GICPFactor<TConfig> gicp_factor_{};
  // Error
  GICPError<TConfig> gicp_error_{};
};
}  // namespace tam::core::state
