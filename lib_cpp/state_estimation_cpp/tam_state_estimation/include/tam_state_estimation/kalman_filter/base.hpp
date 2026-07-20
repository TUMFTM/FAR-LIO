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
#include <iostream>
#include <map>
#include <memory>
#include <string>
#include <vector>

// type definitions
#include "tam_state_estimation/helper/types.hpp"
#include "tum_types_cpp/common.hpp"
#include "tum_types_cpp/control.hpp"

// Param manager
#include "param_management_cpp/param_value_manager.hpp"

// helper functions
#include "tum_helpers_cpp/geometry/geometry.hpp"

// State Estimation constants / template input
#include "state_estimation_constants/EKF_2D.hpp"
#include "state_estimation_constants/EKF_3D.hpp"

namespace tam::core::state {
template <class TConfig>
class KFBase
{
public:
  /**
   * @brief Constructor
   */
  KFBase()
  {
    // ensure that all matrices are set to zero
    x_.setZero();
    x_pred_.setZero();
    P_.setZero();
    P_pred_.setZero();
    R_.setZero();
    R_adaptive_.setZero();
    R_decay_.setZero();
    Q_.setZero();
    H_.setZero();
    H_full_.setZero();
    K_.setZero();
    residuals_.setZero();
    residuals_raw_.setZero();
    outlier_bound_.setZero();
    fusion_vec_.setZero();
  };

  /**
   * @brief update the measurement covariance matrix
   *
   * @param[in] valid_map     - map that contains the current sensor valid bits
   *                            measurement covariance matrix
   */
  void update_measurement_covariance_matrix(const tam::types::state::unordered_identifier_map<bool>& valid_map)
  {
    // get the configured Measurement covarinace matrix
    const Eigen::Map<const Eigen::VectorXd> R_configured(
      kalman_filter_params_.R_init.data(), kalman_filter_params_.R_init.size());

    // clang-format off
    for (int i = 0; i < R_adaptive_.size(); ++i) {
      double covariance_limit = kalman_filter_params_.covariance_adaption_limit;
      double covariance_decay = kalman_filter_params_.covariance_adaption_decay;
      if (i >= TConfig::MEASUREMENT_VECTOR_OFFSET_ORIENTATION) {
        covariance_limit /= 100;
        covariance_decay /= 5;
      }
      // ensure no elements are set to zero
      if (std::abs(R_adaptive_(i)) < R_configured(i)) R_adaptive_(i) = R_configured(i);

      // decrease the variance that was set to the limit after a change in the valid bits
      if (R_(i, i) > R_adaptive_[i]) {
        if (R_(i, i) > covariance_limit && R_adaptive_[i] < covariance_limit) {
          R_(i, i) = covariance_limit;
          R_decay_[i] = static_cast<uint16_t>(covariance_limit / covariance_decay);
        }
        if (((R_(i, i) < covariance_limit || R_adaptive_[i] < covariance_limit) && R_(i, i) > covariance_decay * 2) && R_decay_[i] > 0) {  // NOLINT
          R_(i, i) -= covariance_decay;
          R_decay_[i]--;
        } else {
          R_(i, i) = R_adaptive_[i];
        }
      } else {
        R_(i, i) = R_adaptive_[i];
      }
    }

    // check all position valid bits
    for (uint8_t i = 0; i < TConfig::NUM_POS_MEASUREMENT; ++i) {
      const tam::types::state::measurements::identifier key = {tam::types::state::measurements::POS, i};
      if (valid_map.at(key) != previous_valid_map_.at(key)) {
        // set all measurement covariances to the limit if a valid bit changes
        R_(i * TConfig::POS_MEASUREMENT_VECTOR_SIZE + TConfig::MEASUREMENT_POS_X_M, i * TConfig::POS_MEASUREMENT_VECTOR_SIZE + TConfig::MEASUREMENT_POS_X_M) = kalman_filter_params_.covariance_adaption_limit; // NOLINT
        R_(i * TConfig::POS_MEASUREMENT_VECTOR_SIZE + TConfig::MEASUREMENT_POS_Y_M, i * TConfig::POS_MEASUREMENT_VECTOR_SIZE + TConfig::MEASUREMENT_POS_Y_M) = kalman_filter_params_.covariance_adaption_limit; // NOLINT
        if constexpr (TConfig::POS_MEASUREMENT_VECTOR_SIZE > 2) {
          R_(i * TConfig::POS_MEASUREMENT_VECTOR_SIZE + TConfig::MEASUREMENT_POS_Z_M, i * TConfig::POS_MEASUREMENT_VECTOR_SIZE + TConfig::MEASUREMENT_POS_Z_M) = kalman_filter_params_.covariance_adaption_limit; // NOLINT
        }
      }
    }

    // check all orientation valid bits
    for (uint8_t i = 0; i < TConfig::NUM_ORIENTATION_MEASUREMENT; ++i) {
      const tam::types::state::measurements::identifier key = {tam::types::state::measurements::ORIENTATION, i};
      if (valid_map.at(key) != previous_valid_map_.at(key)) {
        // set all measurement covariances to the limit if a valid bit changes
        const int orientation_offset = TConfig::MEASUREMENT_VECTOR_OFFSET_ORIENTATION + i * TConfig::ORIENTATION_MEASUREMENT_VECTOR_SIZE; // NOLINT
        R_(orientation_offset + TConfig::MEASUREMENT_PSI_RAD, orientation_offset + TConfig::MEASUREMENT_PSI_RAD) = kalman_filter_params_.covariance_adaption_limit / 100; // NOLINT
        if constexpr (TConfig::ORIENTATION_MEASUREMENT_VECTOR_SIZE > 1) {
          R_(orientation_offset + TConfig::MEASUREMENT_PHI_RAD, orientation_offset + TConfig::MEASUREMENT_PHI_RAD) = kalman_filter_params_.covariance_adaption_limit / 100; // NOLINT
          R_(orientation_offset + TConfig::MEASUREMENT_THETA_RAD, orientation_offset + TConfig::MEASUREMENT_THETA_RAD) = kalman_filter_params_.covariance_adaption_limit / 100; // NOLINT
        }
      }
    }
    // clang-format on

    previous_valid_map_ = valid_map;
  }

  /**
   * @brief set the covariance matricies defined in the param handler
   */
  void set_covariance_matricies(void)
  {
    // update parameters
    if (param_manager_->get_state_hash() != previous_param_state_hash_) {
      this->declare_and_update_parameters();
    }

    // set the Measurement Noise Covariance Matrix
    R_.diagonal() =
      Eigen::Map<Eigen::VectorXd>(kalman_filter_params_.R_init.data(), kalman_filter_params_.R_init.size());
    R_adaptive_ = Eigen::Map<Eigen::VectorXd>(kalman_filter_params_.R_init.data(), kalman_filter_params_.R_init.size());

    // set the Process Noise Covariance Matrix
    Q_.diagonal() = Eigen::Map<Eigen::VectorXd>(kalman_filter_params_.Q.data(), kalman_filter_params_.Q.size());

    // set the outlier bound vector to only load the data from the param manager once
    outlier_bound_ = Eigen::Map<Eigen::VectorXd>(
      kalman_filter_params_.outlier_bounds.data(), kalman_filter_params_.outlier_bounds.size());
  };

  /**
   * @brief update the measurement matrix to only fuse sensors that have been updated
   *
   * @param[in] fusion_vec    - Vector indicating which sensors have been updated
   *                            (1: update, 0: not updated)
   */
  void update_measurement_matrix(
    const Eigen::Ref<const Eigen::Vector<double, TConfig::MEASUREMENT_VECTOR_SIZE>>& fusion_vec)
  {
    // extraction matrix which only uses the sensors configured and updated
    // buffer the fusion vector for the update step of the filter
    fusion_vec_ = fusion_vec;

    // update the measurement matrix such that we only use updated measurements in the
    // update step of the Kalman filter (diagonal row-scaling of H_full_)
    H_.noalias() = fusion_vec.asDiagonal() * H_full_;
  };

  /**
   * @brief set the initial valid bits for the sensor status and covariance adaptation
   *
   * @param[in] valid_map     - map that contains the current sensor valid bits
   *                            measurement covariance matrix
   */
  void set_initial_valid_map(const tam::types::state::unordered_identifier_map<bool>& valid_map)
  {
    previous_valid_map_ = valid_map;
  };

  /**
   * @brief set the state vector externally to initialize the extended kalman filter
   *
   * @param[in] x             - Vector representing the state vector created from raw measurements
   */
  void set_state_vector(const Eigen::Ref<const Eigen::Vector<double, TConfig::STATE_VECTOR_SIZE>>& x) { x_ = x; };

  /**
   * @brief sets the position covariance of one input
   *
   * @param[in] covariance        - Eigen::Vector(POS_MEASUREMENT_VECTOR_SIZE):
   *                                covariance of the position measurements
   * @param[in] pos_num           - uint8_t:
   *                                containing the number of the position input [0-N]
   */
  void set_position_covariance(
    const Eigen::Ref<const Eigen::Vector<double, TConfig::POS_MEASUREMENT_VECTOR_SIZE>>& covariance, uint8_t pos_num)
  {
    // set the variance in the diagonal of the measurement noise covariance matrix for the position
    // clang-format off
    if (pos_num >= TConfig::NUM_POS_MEASUREMENT)
      throw std::invalid_argument(
        "[StateEstimationCPP]: Position input " + std::to_string(pos_num) + " does not exist");
    if (covariance.array().isNaN().any()) {
      // If any element is NaN, set all elements in the specified segment to zer
      R_adaptive_.segment(pos_num * TConfig::POS_MEASUREMENT_VECTOR_SIZE, TConfig::POS_MEASUREMENT_VECTOR_SIZE).setZero(); // NOLINT
    } else {
      // If no element is NaN, assign the covariance vector to the specified segment
      R_adaptive_.segment(pos_num * TConfig::POS_MEASUREMENT_VECTOR_SIZE, TConfig::POS_MEASUREMENT_VECTOR_SIZE) = covariance; // NOLINT
    }
    // clang-format on
  };

  /**
   * @brief sets the orientation covariance of one input
   *
   * @param[in] covariance        - Eigen::Vector(ORIENTATION_MEASUREMENT_VECTOR_SIZE):
   *                                covariance of the orientation measurements
   * @param[in] orientation_num   - uint8_t:
   *                                containing the number of the orientation input [0-N]
   */
  void set_orientation_covariance(
    const Eigen::Ref<const Eigen::Vector<double, TConfig::ORIENTATION_MEASUREMENT_VECTOR_SIZE>>& covariance,
    uint8_t orientation_num)
  {
    // set the variance in the diagonal of the measurement noise covariance matrix for the orientation
    // clang-format off
    if (orientation_num >= TConfig::NUM_ORIENTATION_MEASUREMENT)
      throw std::invalid_argument(
        "[StateEstimationCPP]: Position input " + std::to_string(orientation_num) + " does not exist");
    if (covariance.array().isNaN().any()) {
      // If any element is NaN, set all elements in the specified segment to zero
      R_adaptive_.segment(TConfig::MEASUREMENT_VECTOR_OFFSET_ORIENTATION + orientation_num * TConfig::ORIENTATION_MEASUREMENT_VECTOR_SIZE, TConfig::ORIENTATION_MEASUREMENT_VECTOR_SIZE).setZero(); // NOLINT
    } else {
      // If no element is NaN, assign the covariance vector to the specified segment
      R_adaptive_.segment(TConfig::MEASUREMENT_VECTOR_OFFSET_ORIENTATION + orientation_num * TConfig::ORIENTATION_MEASUREMENT_VECTOR_SIZE, TConfig::ORIENTATION_MEASUREMENT_VECTOR_SIZE) = covariance; // NOLINT
    }
    // clang-format on
  };

  /**
   * @brief Get state vector of the Kalman Filter
   *
   * @param[out]                  - Eigen::Vector:
   *                                Kalman filter state vector
   */
  const Eigen::Vector<double, TConfig::STATE_VECTOR_SIZE> get_state_vector(void) const { return x_; };

  /**
   * @brief Get the predicted covariance matrix of the Kalman Filter
   *
   * @param[out]                  - Eigen::Matrix:
   *                                Kalman filter covariance matrix
   */
  const Eigen::Matrix<double, TConfig::STATE_VECTOR_SIZE, TConfig::STATE_VECTOR_SIZE>& get_covariance_matrix(void) const
  {
    return P_;
  };

  /**
   * @brief Get a boolean indicating if the adaptive measurement covariance is enabled
   *
   * @param[out]                  - bool:
   *                               boolean if adaptive measurement covariance is enabled
   */
  bool get_enable_adaptive_measurement_covariance(void) const
  {
    return kalman_filter_params_.enable_measurement_covariance_adaptation;
  };

  /**
   * @brief Get pointer on param manager
   *
   * @param[out]                  - std::shared_ptr<tam::interfaces::ParamManagerBase>
   *                               pointer on the param manager
   */
  tam::pmg::ParamValueManager::SharedPtr get_param_handler(void) { return param_manager_; };

  /**
   * @brief Declare and update the parameters of the kalman filter
   */
  void declare_and_update_parameters(void)
  {
    // clang-format off
    kalman_filter_params_.enable_mahalanobis_outlier_rejection = param_manager_->declare_and_get_value("enable.mahalanobis_outlier_rejection", false, tam::pmg::ParameterType::BOOL, "Use the Mahalanobis distance instead of fixed per-element bounds to reject measurement outliers").as_bool(); // NOLINT
    kalman_filter_params_.enable_measurement_covariance_adaptation = param_manager_->declare_and_get_value("enable.measurement_covariance_adaptation", true, tam::pmg::ParameterType::BOOL, "Continuously adapt the measurement noise covariance R to the covariance reported by each measurement source").as_bool(); // NOLINT
    kalman_filter_params_.covariance_adaption_limit = param_manager_->declare_and_get_value("kalman_filter.covariance_adaption.limit", 1.5, tam::pmg::ParameterType::DOUBLE, "Covariance value that R is raised to after a sudden jump, before it decays back toward the target covariance").as_double(); // NOLINT
    kalman_filter_params_.covariance_adaption_decay = param_manager_->declare_and_get_value("kalman_filter.covariance_adaption.decay", 0.00165, tam::pmg::ParameterType::DOUBLE, "Amount by which R is reduced each update step while it decays from the limit back toward the target covariance").as_double(); // NOLINT
    kalman_filter_params_.R_init = param_manager_->declare_and_get_value("kalman_filter.R_init", std::vector<double>(TConfig::MEASUREMENT_VECTOR_SIZE, 1.0), tam::pmg::ParameterType::DOUBLE_ARRAY, "Initial (and minimum) diagonal of the measurement noise covariance R of the EKF").as_double_array(); // NOLINT
    kalman_filter_params_.Q = param_manager_->declare_and_get_value("kalman_filter.Q", std::vector<double>(TConfig::Q_VECTOR_SIZE, 1.0), tam::pmg::ParameterType::DOUBLE_ARRAY, "Process noise covariance Q diagonal of the EKF").as_double_array(); // NOLINT
    kalman_filter_params_.outlier_bounds = param_manager_->declare_and_get_value("kalman_filter.outlier_bounds", std::vector<double>(TConfig::MEASUREMENT_VECTOR_SIZE, 1.0), tam::pmg::ParameterType::DOUBLE_ARRAY, "Per-element bounds used to reject measurement outliers when Mahalanobis rejection is disabled").as_double_array(); // NOLINT
    previous_param_state_hash_ = param_manager_->get_state_hash();
    // clang-format on
  };

  // virtual functions
  /**
   * @brief Prediction step of the EKF
   *
   * @param[in] u             - Input vector
   */
  virtual void predict(const Eigen::Ref<const Eigen::Vector<double, TConfig::INPUT_VECTOR_SIZE>>& u) = 0;

  /**
   * @brief Correction step of the EKF
   *
   * @param[in] z             - Measurement vector
   */
  virtual void update(const Eigen::Ref<const Eigen::Vector<double, TConfig::MEASUREMENT_VECTOR_SIZE>>& z) = 0;

  /**
   * @brief get all debug values of the kalman filter
   *
   * @param[out]                  - std::unordered_map:
   *                                residuals of the Extended Kalman Filter
   */
  virtual const tam::types::state::unordered_identifier_map<std::vector<std::pair<std::string, double>>>& get_debug(
    void) = 0;

protected:
  // Variables
  /**
   * @brief State vector of the Kalman Filter
   */
  Eigen::Vector<double, TConfig::STATE_VECTOR_SIZE> x_{};

  /**
   * @brief State vector of the Kalman Filter after the prediction step
   */
  Eigen::Vector<double, TConfig::STATE_VECTOR_SIZE> x_pred_{};

  /**
   * @brief Covariance matrix of the Kalman Filter
   */
  Eigen::Matrix<double, TConfig::STATE_VECTOR_SIZE, TConfig::STATE_VECTOR_SIZE> P_{};

  /**
   * @brief Covariance matrix of the Kalman Filter after the prediction step
   */
  Eigen::Matrix<double, TConfig::STATE_VECTOR_SIZE, TConfig::STATE_VECTOR_SIZE> P_pred_{};

  /**
   * @brief Measurement noise covariance matrix of the Kalman Filter
   */
  Eigen::Matrix<double, TConfig::MEASUREMENT_VECTOR_SIZE, TConfig::MEASUREMENT_VECTOR_SIZE> R_{};

  /**
   * @brief Vector containing the diagonal elements of the adaptive measurement noise covariance matrix
   */
  Eigen::Vector<double, TConfig::MEASUREMENT_VECTOR_SIZE> R_adaptive_{};

  /**
   * @brief Vector containing the number of decay steps needed to be performed on the diagonal elements
   */
  Eigen::Vector<uint16_t, TConfig::MEASUREMENT_VECTOR_SIZE> R_decay_{};

  /**
   * @brief Process noise covariance matrix of the Kalman Filter
   */
  Eigen::Matrix<double, TConfig::Q_VECTOR_SIZE, TConfig::Q_VECTOR_SIZE> Q_{};

  /**
   * @brief Measurement matrix of the Kalman Filter (only uses fresh measurements)
   */
  Eigen::Matrix<double, TConfig::MEASUREMENT_VECTOR_SIZE, TConfig::STATE_VECTOR_SIZE> H_{};

  /**
   * @brief Full Measurement matrix of the Kalman Filter
   */
  Eigen::Matrix<double, TConfig::MEASUREMENT_VECTOR_SIZE, TConfig::STATE_VECTOR_SIZE> H_full_{};

  /**
   * @brief Kalman Gain
   */
  Eigen::Matrix<double, TConfig::STATE_VECTOR_SIZE, TConfig::MEASUREMENT_VECTOR_SIZE> K_{};

  /**
   * @brief Innovation/ Measurement Residuals of the Kalman Filter after the outlier rejection
   */
  Eigen::Vector<double, TConfig::MEASUREMENT_VECTOR_SIZE> residuals_{};

  /**
   * @brief Innovation/ Measurement Residuals of the Kalman Filter
   */
  Eigen::Vector<double, TConfig::MEASUREMENT_VECTOR_SIZE> residuals_raw_{};

  /**
   * @brief Maximum residual (everything above this threshold is detected as outlier)
   */
  Eigen::Vector<double, TConfig::MEASUREMENT_VECTOR_SIZE> outlier_bound_{};

  /**
   * @brief Vector indicating which sensors have been updated and is valid
   *        (1: update, 0: not updated)
   */
  Eigen::Vector<double, TConfig::MEASUREMENT_VECTOR_SIZE> fusion_vec_{};

  /**
   * @brief map containing the valid bits of the previous step
   */
  tam::types::state::unordered_identifier_map<bool> previous_valid_map_{};

  /**
   * @brief map containing the debug outputs of the kalman filter
   */
  tam::types::state::unordered_identifier_map<std::vector<std::pair<std::string, double>>> debug_output_{};

  /**
   * @brief Variable containing the previous parameter state hash
   */
  std::size_t previous_param_state_hash_{0};

  /**
   * @brief Struct containing the parameters for the kalman filter
   */
  tam::types::state::config::kalman_filter kalman_filter_params_{};

  /**
   * @brief Pointer on the param manager
   */
  tam::pmg::ParamValueManager::SharedPtr param_manager_ = std::make_shared<tam::pmg::ParamValueManager>();
};
}  // namespace tam::core::state
