/*
 * Copyright 2023 Marcel Weinmann, Maximilian Leitenstern
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

#include <eigen3/Eigen/Dense>

// type definitions
#include "tum_types_cpp/common.hpp"

// helper functions
#include "tum_helpers_cpp/rotations.hpp"

// State Estimation constants / template input
#include "state_estimation_constants/EKF_2D.hpp"
#include "state_estimation_constants/EKF_3D.hpp"

namespace tam::core::state::outlier_rejection {
/**
 * @brief Calculate the squared distance between a explict and the mean of a measurement
 *
 * @param[in] measurement       - Explicit measurement
 *
 * @param[in] mean              - Mean of the measurement over window size N
 *
 * @param[out]                  - Squared distance
 */
inline Eigen::VectorXd squared_distance(const Eigen::VectorXd& measurement, const Eigen::VectorXd& mean)
{
  return (measurement - mean).cwiseProduct(measurement - mean);
}

inline Eigen::Vector3d squared_distance(
  const tam::types::common::Vector3D<double>& measurement, const tam::types::common::Vector3D<double>& mean)
{
  const tam::types::common::Vector3D difference = measurement - mean;
  Eigen::Vector3d out;
  out << difference.x * difference.x, difference.y * difference.y, difference.z * difference.z;
  return out;
}

inline Eigen::Vector3d squared_distance(
  const Eigen::Vector3d& measurement, const tam::types::common::Vector3D<double>& mean)
{
  Eigen::Vector3d difference;
  difference << measurement[0] - mean.x, measurement[1] - mean.y, measurement[2] - mean.z;
  return difference.cwiseProduct(difference);
}

/**
 * @brief Performs an outlier detection and rejection using box shaped outlier bounds
 *
 * @param[in] residuals         - Raw residual vector
 *
 * @param[in] outlier_bound     - Box outlier bounds
 *
 * @param[out]                  - Residual vector without outliers
 */
inline Eigen::VectorXd box_outlier_rejection(const Eigen::VectorXd& residuals, const Eigen::VectorXd& outlier_bound)
{
  const Eigen::VectorXd signs = residuals.array().sign();
  const Eigen::VectorXd residuals_filtered = residuals.cwiseAbs().cwiseMin(outlier_bound).cwiseProduct(signs);
  return residuals_filtered;
}

/**
 * @brief Performs an outlier detection and rejection using the mahalanobis distance
 *
 * @param[in] residuals         - Raw residual vector
 *
 * @param[in] covariance_diag   - Diagonal elements of the covariance matrix
 *
 * @param[in] x                 - Current state vector
 *
 * @param[out]                  - Residual vector without outliers
 */
template <typename TConfig>
inline Eigen::Vector<double, TConfig::MEASUREMENT_VECTOR_SIZE> mahalanobis_outlier_rejection(
  const Eigen::Ref<Eigen::Vector<double, TConfig::MEASUREMENT_VECTOR_SIZE>> residuals,
  const Eigen::Ref<Eigen::Vector<double, TConfig::MEASUREMENT_VECTOR_SIZE>> covariance_diag,
  const Eigen::Ref<Eigen::Vector<double, TConfig::STATE_VECTOR_SIZE>> x);

/**
 * @brief Performs an outlier detection and rejection using the mahalanobis distance
 *        for the EKF operating in 2D
 *
 * @param[in] residuals         - Raw residual vector
 *
 * @param[in] covariance_diag   - Diagonal elements of the covariance matrix
 *
 * @param[in] x                 - Current state vector
 *
 * @param[out]                  - Residual vector without outliers
 */
template <>
inline Eigen::Vector<double, tam::core::state::EKF_2D::MEASUREMENT_VECTOR_SIZE>
mahalanobis_outlier_rejection<tam::core::state::EKF_2D>(
  const Eigen::Ref<Eigen::Vector<double, tam::core::state::EKF_2D::MEASUREMENT_VECTOR_SIZE>> residuals,
  const Eigen::Ref<Eigen::Vector<double, tam::core::state::EKF_2D::MEASUREMENT_VECTOR_SIZE>> covariance_diag,
  const Eigen::Ref<Eigen::Vector<double, tam::core::state::EKF_2D::STATE_VECTOR_SIZE>> x)
{
  using TConfig = tam::core::state::EKF_2D;
  Eigen::Matrix<double, TConfig::MEASUREMENT_VECTOR_SIZE, TConfig::MEASUREMENT_VECTOR_SIZE> covariance{};
  Eigen::Vector<double, TConfig::MEASUREMENT_VECTOR_SIZE> residuals_filtered = residuals;
  covariance.diagonal() = covariance_diag;

  Eigen::Vector3d euler_rotation;
  euler_rotation << 0.0, 0.0, x[TConfig::STATE_PSI_RAD];

  for (int i = 0; i < TConfig::NUM_POS_MEASUREMENT; ++i) {
    const uint8_t vec_offset = i * TConfig::POS_MEASUREMENT_VECTOR_SIZE;
    Eigen::Vector3d residual_vector = Eigen::Vector3d::Zero();

    // extract positional measurements
    residual_vector.segment(0, 2) = residuals_filtered.segment(vec_offset, 2);

    // rotate the residuals into the vehicle coordinate frame
    const Eigen::Vector2d residual_vector_rotated =
      tam::helpers::euler_rotations::math::rotate_inv_rpy(residual_vector, euler_rotation).segment(0, 2);

    // calculate the Mahalanobis distance
    const Eigen::Matrix<double, 2, 2> covariance_position = covariance.block(vec_offset, vec_offset, 2, 2).inverse();
    const double mahalanobis_distance_position =
      std::sqrt(residual_vector_rotated.transpose() * covariance_position * residual_vector_rotated);

    // if the outlier bound was violated normalize the residuals wrt the mahalanobis distance
    // otherwise just return the original residual
    if (mahalanobis_distance_position > 1.0) {
      residual_vector.segment(0, 2) = residual_vector_rotated / mahalanobis_distance_position;
      residuals_filtered.segment(vec_offset, 2) =
        tam::helpers::euler_rotations::math::rotate_ypr(residual_vector, euler_rotation).segment(0, 2);
    }
  }

  for (int i = 0; i < TConfig::NUM_ORIENTATION_MEASUREMENT; ++i) {
    const uint8_t vec_offset =
      i * TConfig::ORIENTATION_MEASUREMENT_VECTOR_SIZE + TConfig::MEASUREMENT_VECTOR_OFFSET_ORIENTATION;
    // use box outlier detection for angles
    residuals_filtered.segment(vec_offset, 1) =
      box_outlier_rejection(residuals_filtered.segment(vec_offset, 1), covariance_diag.segment(vec_offset, 1));
  }

  for (int i = 0; i < TConfig::NUM_VEL_MEASUREMENT; ++i) {
    const uint8_t vec_offset = i * TConfig::VEL_MEASUREMENT_VECTOR_SIZE + TConfig::MEASUREMENT_VECTOR_OFFSET_VEL;
    // extract velocity measurements
    const Eigen::Vector2d residual_vector = residuals_filtered.segment(vec_offset, 2);

    // calculate the Mahalanobis distance
    const Eigen::Matrix<double, 2, 2> covariance_orientation = covariance.block(vec_offset, vec_offset, 2, 2).inverse();
    const double mahalanobis_distance_velocity =
      std::sqrt(residual_vector.transpose() * covariance_orientation * residual_vector);

    // if the outlier bound was violated normalize the residuals wrt the mahalanobis distance
    // otherwise just return the original residual
    if (mahalanobis_distance_velocity > 1.0) {
      residuals_filtered.segment(vec_offset, 2) = residual_vector / mahalanobis_distance_velocity;
    }
  }

  return residuals_filtered;
}

/**
 * @brief Performs an outlier detection and rejection using the mahalanobis distance
 *        for Kalman filter operating in 3D
 *
 * @param[in] residuals         - Raw residual vector
 *
 * @param[in] covariance_diag   - Diagonal elements of the covariance matrix
 *
 * @param[in] x                 - Current state vector
 *
 * @param[out]                  - Residual vector without outliers
 */
template <>
inline Eigen::Vector<double, tam::core::state::EKF_3D::MEASUREMENT_VECTOR_SIZE>
mahalanobis_outlier_rejection<tam::core::state::EKF_3D>(
  const Eigen::Ref<Eigen::Vector<double, tam::core::state::EKF_3D::MEASUREMENT_VECTOR_SIZE>> residuals,
  const Eigen::Ref<Eigen::Vector<double, tam::core::state::EKF_3D::MEASUREMENT_VECTOR_SIZE>> covariance_diag,
  const Eigen::Ref<Eigen::Vector<double, tam::core::state::EKF_3D::STATE_VECTOR_SIZE>> x)
{
  using TConfig = tam::core::state::EKF_3D;
  Eigen::Matrix<double, TConfig::MEASUREMENT_VECTOR_SIZE, TConfig::MEASUREMENT_VECTOR_SIZE> covariance{};
  Eigen::Vector<double, TConfig::MEASUREMENT_VECTOR_SIZE> residuals_filtered = residuals;
  covariance.diagonal() = covariance_diag;

  Eigen::Vector3d euler_rotation;
  euler_rotation << x[TConfig::STATE_PHI_RAD], x[TConfig::STATE_THETA_RAD], x[TConfig::STATE_PSI_RAD];

  for (int i = 0; i < TConfig::NUM_POS_MEASUREMENT; ++i) {
    const uint8_t vec_offset = i * TConfig::POS_MEASUREMENT_VECTOR_SIZE;
    // extract positional measurements
    Eigen::Vector3d residual_vector = residuals_filtered.segment(vec_offset, 3);

    // rotate the residuals into the vehicle coordinate frame
    const Eigen::Vector3d residual_vector_rotated =
      tam::helpers::euler_rotations::math::rotate_inv_rpy(residual_vector, euler_rotation);

    // calculate the Mahalanobis distance
    const Eigen::Matrix<double, 3, 3> covariance_position = covariance.block(vec_offset, vec_offset, 3, 3).inverse();
    const double mahalanobis_distance_position =
      std::sqrt(residual_vector_rotated.transpose() * covariance_position * residual_vector_rotated);

    // if the outlier bound was violated normalize the residuals wrt the mahalanobis distance
    // otherwise just return the original residual
    if (mahalanobis_distance_position > 1.0) {
      residual_vector = residual_vector_rotated / mahalanobis_distance_position;
      residuals_filtered.segment(vec_offset, 3) =
        tam::helpers::euler_rotations::math::rotate_ypr(residual_vector, euler_rotation);
    }
  }

  for (int i = 0; i < TConfig::NUM_ORIENTATION_MEASUREMENT; ++i) {
    const uint8_t vec_offset =
      i * TConfig::ORIENTATION_MEASUREMENT_VECTOR_SIZE + TConfig::MEASUREMENT_VECTOR_OFFSET_ORIENTATION;
    // extract orientation measurements
    const Eigen::Vector3d residual_vector = residuals_filtered.segment(vec_offset, 3);

    // calculate the Mahalanobis distance
    const Eigen::Matrix<double, 3, 3> covariance_orientation = covariance.block(vec_offset, vec_offset, 3, 3).inverse();
    const double mahalanobis_distance_orientation =
      std::sqrt(residual_vector.transpose() * covariance_orientation * residual_vector);

    // if the outlier bound was violated normalize the residuals wrt the mahalanobis distance
    // otherwise just return the original residual
    if (mahalanobis_distance_orientation > 1.0) {
      residuals_filtered.segment(vec_offset, 3) = residual_vector / mahalanobis_distance_orientation;
    }
  }

  for (int i = 0; i < TConfig::NUM_VEL_MEASUREMENT; ++i) {
    const uint8_t vec_offset = i * TConfig::VEL_MEASUREMENT_VECTOR_SIZE + TConfig::MEASUREMENT_VECTOR_OFFSET_VEL;
    // extract velocity measurements
    const Eigen::Vector3d residual_vector = residuals_filtered.segment(vec_offset, 3);

    // calculate the Mahalanobis distance
    const Eigen::Matrix<double, 3, 3> covariance_orientation = covariance.block(vec_offset, vec_offset, 3, 3).inverse();
    const double mahalanobis_distance_velocity =
      std::sqrt(residual_vector.transpose() * covariance_orientation * residual_vector);

    // if the outlier bound was violated normalize the residuals wrt the mahalanobis distance
    // otherwise just return the original residual
    if (mahalanobis_distance_velocity > 1.0) {
      residuals_filtered.segment(vec_offset, 3) = residual_vector / mahalanobis_distance_velocity;
    }
  }

  return residuals_filtered;
}
}  // namespace tam::core::state::outlier_rejection
