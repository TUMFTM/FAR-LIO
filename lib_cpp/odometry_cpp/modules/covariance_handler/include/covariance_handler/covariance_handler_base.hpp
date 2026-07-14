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
#include <eigen3/Eigen/Core>
#include <memory>
#include <sophus/se3.hpp>
#include <vector>

#include "odometry_base/odometry_base.hpp"
#include "robust_kernel/robust_kernel.hpp"
namespace tam::core::state
{
template <typename TConfig>
class CovarianceHandler
: public OdometryBase<TConfig, types::CovarianceConfig, types::CovarianceDebug>
{
public:
  virtual std::array<float, 36> get_pose_covariance(
    [[maybe_unused]] const Sophus::SE3f & pose,
    [[maybe_unused]] const std::vector<types::Correspondence<TConfig>> & correspondences,
    [[maybe_unused]] const float kernel_scale) = 0;
  std::array<float, 36> get_tangent_covariance() const
  {
    // Return config values for minimum covariance as twist covariance
    Eigen::Matrix<float, 6, 6> cov_matrix = Eigen::Matrix<float, 6, 6>::Zero();
    std::array<float, 6> min_cov = {
      static_cast<float>(this->config_.min_cov_linear_twist[0]),
      static_cast<float>(this->config_.min_cov_linear_twist[1]),
      static_cast<float>(this->config_.min_cov_linear_twist[2]),
      static_cast<float>(this->config_.min_cov_angular_twist[0]),
      static_cast<float>(this->config_.min_cov_angular_twist[1]),
      static_cast<float>(this->config_.min_cov_angular_twist[2])};
    return matrix2array(cov_matrix, min_cov);
  }

protected:
  // Inherit constructor from OdometryBase for param manager and logger
  CovarianceHandler(tam::pmg::ParamReferenceManager * pmg, tam::tsl::ReferenceLogger * logger)
  : OdometryBase<TConfig, types::CovarianceConfig, types::CovarianceDebug>(pmg, logger)
  {
    this->set_config(pmg);
    this->set_logging(logger);
    // Additional initialization
  }
  // Inherit constructor from OdometryBase for config and debug object
  CovarianceHandler(const types::CovarianceConfig & config, const types::CovarianceDebug & debug)
  : OdometryBase<TConfig, types::CovarianceConfig, types::CovarianceDebug>(config, debug)
  {
    // Additional initialization
  }
  /**
   * @brief Set the configuration of the covariance handler from the param manager
   * @param [in] pmg                Param manager
   */
  void set_config(tam::pmg::ParamReferenceManager * pmg) override
  {
    // clang-format off
    pmg->declare_parameter("covariance.min_cov_translation", &this->config_.min_cov_translation, std::vector<double>{0.01, 0.01, 0.01}, tam::pmg::ParameterType::DOUBLE_ARRAY, "Min translation covariance diagonal [x, y, z] (m^2)");  // NOLINT
    pmg->declare_parameter("covariance.min_cov_orientation", &this->config_.min_cov_orientation, std::vector<double>{0.001, 0.001, 0.001}, tam::pmg::ParameterType::DOUBLE_ARRAY, "Min orientation covariance diagonal [roll, pitch, yaw] (rad^2)");  // NOLINT
    pmg->declare_parameter("covariance.min_cov_linear_twist", &this->config_.min_cov_linear_twist, std::vector<double>{0.1, 0.1, 0.1}, tam::pmg::ParameterType::DOUBLE_ARRAY, "Min linear twist covariance diagonal [x, y, z] ((m/s)^2)");  // NOLINT
    pmg->declare_parameter("covariance.min_cov_angular_twist", &this->config_.min_cov_angular_twist, std::vector<double>{0.01, 0.01, 0.01}, tam::pmg::ParameterType::DOUBLE_ARRAY, "Min angular twist covariance diagonal [x, y, z] ((rad/s)^2)");  // NOLINT
    // clang-format on
  }
  /**
   * @brief Register the debug variables with the logger
   * @param [in] logger             Logger
   */
  void set_logging(tam::tsl::ReferenceLogger * logger) const override
  {
    logger->log("covariance/pose_covariance_time", &this->debug_.pose_covariance_time);
    logger->log("covariance/twist_covariance_time", &this->debug_.twist_covariance_time);
  }

protected:
  /**
   * @brief Convert Eigen matrix to array and check for minimum covariance values
   * @param [in] cov_matrix         Eigen matrix
   * @param [in] min_cov            Minimum values for the covariance diagonal
   * @return                        Array of floats
   */
  std::array<float, 36> matrix2array(
    Eigen::Matrix<float, 6, 6> & cov_matrix, const std::array<float, 6> & min_cov) const
  {
    // Initialize array
    std::array<float, 36> cov_array = {};
    // Convert matrix to array
    for (int i = 0; i < 6; ++i) {
      for (int j = 0; j < 6; ++j) {
        if (i == j) {
          cov_array[6 * i + j] = std::max(cov_matrix(i, j), min_cov[i]);
        } else {
          cov_array[6 * i + j] = cov_matrix(i, j);
        }
      }
    }
    return cov_array;
  }
  /**
   * @brief Calculate Cramér-Rao lower bound
   * @param [in] correspondences    Correspondences
   * @param [in] pose               Pose
   */
  Eigen::Matrix<float, 6, 6> cramer_rao_bound(
    const std::vector<types::Correspondence<TConfig>> & correspondences, const Sophus::SE3f & pose)
    requires types::HASNORMALCOV<TConfig> && types::HASSEG<TConfig>
  {
    // Initialize variables
    Eigen::Vector3f frame_p = Eigen::Vector3f::Zero();
    Eigen::Vector3f map_p = Eigen::Vector3f::Zero();
    Eigen::Vector3f map_p_normal = Eigen::Vector3f::Zero();
    std::uint32_t segment_id = 0;

    Eigen::Vector<float, 6> jacobian_i = Eigen::Vector<float, 6>::Zero();
    Eigen::Matrix<float, 6, 6> JTJ = Eigen::Matrix<float, 6, 6>::Zero();
    Eigen::Matrix<float, 6, 6> jacobian_i_jacobian_i_T = Eigen::Matrix<float, 6, 6>::Zero();
    // Translation
    Eigen::Vector3f pos = pose.translation();
    // Loop over correspondences
    for (size_t i = 0; i < correspondences.size(); ++i) {
      // Capture correspondence components
      frame_p = correspondences.at(i).frame.pos - pos;
      map_p = correspondences.at(i).map.pos - pos;
      map_p_normal = correspondences.at(i).map.normal;
      segment_id = correspondences.at(i).map.seg;
      // Skip if no segment
      if (map_p_normal.hasNaN() || segment_id == 0) {
        continue;
      }
      jacobian_i.segment(0, 3) = -(frame_p.cross(map_p_normal).transpose());
      jacobian_i.segment(3, 3) = -map_p_normal;
      jacobian_i_jacobian_i_T = jacobian_i * jacobian_i.transpose();
      JTJ += jacobian_i_jacobian_i_T;
    }
    return JTJ.inverse();
  }
};
}  // namespace tam::core::state
