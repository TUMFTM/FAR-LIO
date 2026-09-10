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

#include <algorithm>
#include <cmath>
#include <deque>
#include <eigen3/Eigen/Dense>
#include <iostream>
#include <map>
#include <memory>
#include <optional>
#include <string>
#include <vector>

// type definitions
#include "tam_state_estimation/helper/types.hpp"
#include "tum_types_cpp/common.hpp"
#include "tum_types_cpp/control.hpp"

// general constants
#include "tum_helpers_cpp/constants.hpp"

// Param manager
#include "param_management_cpp/param_value_manager.hpp"

// State Estimation constants / template input
#include "state_estimation_constants/EKF_2D.hpp"
#include "state_estimation_constants/EKF_3D.hpp"

// FIR Filter
#include "tam_state_estimation/helper/fir.hpp"

namespace tam::core::state {
template <class TConfig>
class IMUHandler
{
public:
  /**
   * @brief Constructor
   */
  IMUHandler()
  {
    // set the imu biases to zero
    imu_bias_.setZero();
    initial_imu_bias_.setZero();
    u_bias_fifo_.resize(100);

    declare_and_update_parameters();
  }

  /**
   * @brief Update the raw input vector based on the imu measurements
   *
   * @param[in] u_raw             - Eigen::Vector:
   *                                vector containing the raw IMU measurements
   * @param[in] u_fusion_vec      - Eigen::Vector:
   *                                vector indicating whether a IMU measurement is valid
   * @param[out] u                - Eigen::Vector:
   *                                Processed input vector to the Kalman Filter
   *                                [dPsi_radps, ax_mps2, ay_mps2]
   */
  // clang-format off
  const Eigen::Vector<double, TConfig::INPUT_VECTOR_SIZE> update_input_vector(
    const Eigen::Ref<const Eigen::Vector<double, TConfig::INPUT_VECTOR_SIZE*(TConfig::NUM_IMU_MEASUREMENT + TConfig::NUM_BACKUP_IMU_MEASUREMENT)>>& u_raw, // NOLINT
    const Eigen::Ref<const Eigen::Vector<double, TConfig::INPUT_VECTOR_SIZE*(TConfig::NUM_IMU_MEASUREMENT + TConfig::NUM_BACKUP_IMU_MEASUREMENT)>>& u_fusion_vec, // NOLINT
    std::optional<Eigen::Vector<double, TConfig::INPUT_VECTOR_SIZE*(TConfig::NUM_IMU_MEASUREMENT + TConfig::NUM_BACKUP_IMU_MEASUREMENT)>> u_cov = std::nullopt) // NOLINT
  // clang-format on
  {
    if (param_manager_->get_state_hash() != previous_param_state_hash_) {
      declare_and_update_parameters();
    }
    if (!u_cov.has_value()) {
      u_cov.emplace();
      u_cov->setZero();
    }

    // average the imu data
    Eigen::Vector<double, TConfig::INPUT_VECTOR_SIZE> u = Eigen::Vector<double, TConfig::INPUT_VECTOR_SIZE>::Zero();
    const Eigen::Vector<double, TConfig::INPUT_VECTOR_SIZE> u_average = average_input_vector(u_raw, u_fusion_vec, *u_cov);

    // update the imu bias FiFo buffer
    if (update_bias_) {
      Eigen::Vector<double, TConfig::INPUT_VECTOR_SIZE> u_outlier =
        Eigen::Vector<double, TConfig::INPUT_VECTOR_SIZE>::Zero();
      if (abs(u_average[TConfig::INPUT_DPSI_RADPS]) < 0.05)
        u_outlier[TConfig::INPUT_DPSI_RADPS] = u_average[TConfig::INPUT_DPSI_RADPS];
      if (abs(u_average[TConfig::INPUT_AX_MPS2]) < 1.0)
        u_outlier[TConfig::INPUT_AX_MPS2] = u_average[TConfig::INPUT_AX_MPS2];
      if (abs(u_average[TConfig::INPUT_AY_MPS2]) < 1.0)
        u_outlier[TConfig::INPUT_AY_MPS2] = u_average[TConfig::INPUT_AY_MPS2];

      if constexpr (TConfig::INPUT_VECTOR_SIZE == 6) {
        if (abs(u_average[TConfig::INPUT_DPHI_RADPS]) < 0.05)
          u_outlier[TConfig::INPUT_DPHI_RADPS] = u_average[TConfig::INPUT_DPHI_RADPS];
        if (abs(u_average[TConfig::INPUT_DTHETA_RADPS]) < 0.05)
          u_outlier[TConfig::INPUT_DTHETA_RADPS] = u_average[TConfig::INPUT_DTHETA_RADPS];
        if (abs(u_average[TConfig::INPUT_AZ_MPS2]) < 1.0)
          u_outlier[TConfig::INPUT_AZ_MPS2] = u_average[TConfig::INPUT_AZ_MPS2];
      }

      sum_bias_ += u_outlier;
      if (u_bias_fifo_.size() > 10000) {
        sum_bias_ -= u_bias_fifo_.back();
        u_bias_fifo_.pop_back();
      }
      u_bias_fifo_.push_front(u_outlier);
      imu_bias_ = sum_bias_ / static_cast<double>(u_bias_fifo_.size());

      imu_bias_[TConfig::INPUT_AX_MPS2] = initial_imu_bias_[TConfig::INPUT_AX_MPS2];
      imu_bias_[TConfig::INPUT_AY_MPS2] = initial_imu_bias_[TConfig::INPUT_AY_MPS2];
      if constexpr (TConfig::INPUT_VECTOR_SIZE == 6) {
        imu_bias_[TConfig::INPUT_AZ_MPS2] = initial_imu_bias_[TConfig::INPUT_AZ_MPS2];
      }
    }

    if constexpr (TConfig::INPUT_VECTOR_SIZE == 6) {
      // subtract estimated sensor bias (for the esekf (SOLA. equation 259))
      u = u_average - imu_bias_;
    } else if constexpr (TConfig::INPUT_VECTOR_SIZE == 3) {
      u = u_average;
      // clamp the banking angle to a physically valid range so 1 / cos(banking) and tan(banking)
      // cannot diverge (a real track never banks anywhere near +-90 deg)
      const double banking_rad = std::clamp(-external_orientation_.x, -1.5, 1.5);
      // compensate the banking angle for dpsi_rad
      u[TConfig::INPUT_DPSI_RADPS] /= std::cos(banking_rad);
      // add ay_calibration_offset to all ay_mps2 measurements
      u[TConfig::INPUT_AY_MPS2] -= imu_bias_[TConfig::INPUT_AY_MPS2];
      // compensate banking for ay_mps2
      u[TConfig::INPUT_AY_MPS2] *= std::sin(banking_rad) * std::tan(banking_rad) + std::cos(banking_rad);
      // compensate gravity for ay_mps2
      u[TConfig::INPUT_AY_MPS2] += std::tan(banking_rad) * tam::constants::g_earth;
    } else {
      static_assert(
        TConfig::INPUT_VECTOR_SIZE == 3 || TConfig::INPUT_VECTOR_SIZE == 6, "[IMUHandler]: invalid INPUT_VECTOR_SIZE");
    }
    return u;
  }

  /**
   * @brief Update the raw input vector based on the imu measurements
   *
   * @param[in] u_raw             - Eigen::Vector:
   *                                vector containing the raw IMU measurements
   * @param[out] u_filt           - Eigen::Vector:
   *                                Processed raw input vector to the Kalman Filter
   */
  const Eigen::Vector<double, TConfig::INPUT_VECTOR_SIZE> filter_imu_measurements(
    const Eigen::Ref<const Eigen::Vector<double, TConfig::INPUT_VECTOR_SIZE>>& u_raw, const double imu_num)
  {
    if (initialize_filter_) {
      for (int i = 0; i < TConfig::NUM_IMU_MEASUREMENT + TConfig::NUM_BACKUP_IMU_MEASUREMENT; ++i) {
        // set the Filter Coefficients
        const Eigen::VectorXd imu_filter_coefficients_imu = Eigen::Map<Eigen::VectorXd>(
          imu_handler_params_.filter_coefficients[i].data(), imu_handler_params_.filter_coefficients[i].size());
        for (int vec_pos = 0; vec_pos < TConfig::INPUT_VECTOR_SIZE; ++vec_pos) {
          filter_imu_[i * TConfig::INPUT_VECTOR_SIZE + vec_pos] =
            std::make_unique<tam::core::state::FIR>(imu_filter_coefficients_imu);
        }
      }
      initialize_filter_ = false;
    }

    Eigen::Vector<double, TConfig::INPUT_VECTOR_SIZE> u_filt;
    u_filt.setZero();
    // filter all values
    for (int i = 0; i < TConfig::INPUT_VECTOR_SIZE; ++i) {
      u_filt[i] = filter_imu_[imu_num * TConfig::INPUT_VECTOR_SIZE + i]->step(u_raw[i]);
    }
    return u_filt;
  }

  /**
   * @brief Update the measured sensor biases for the imus to correct the averaged output vector
   *
   * @param[in] imu_biases        - Eigen::Vector:
   *                                vector containing measured biases for all IMU measurements
   */
  void update_sensor_bias(const bool update_bias) { update_bias_ = update_bias; }

  /**
   * @brief set the external orientation prior used to compensate banking for 2D acceleration inputs
   *
   * @param[in] external_orientation - tam::types::common::Vector3D<double>:
   *                                   vector containing the external orientation prior
   *                                   [banking, slope, 0.0]
   */
  void set_external_orientation(const tam::types::common::Vector3D<double>& external_orientation)
  {
    // set the external orientation prior
    external_orientation_ = external_orientation;
  }

  /**
   * @brief set the sensor biases to be compensated in the averaged output vector
   *
   * @param[in] imu_biases        - Eigen::Vector:
   *                                vector containing biases for all IMU measurements
   */
  void set_sensor_bias(const Eigen::Ref<const Eigen::Vector<double, TConfig::INPUT_VECTOR_SIZE>>& imu_bias)
  {
    // set the imu_bias
    initial_imu_bias_ = imu_bias;
    imu_bias_ = imu_bias;
    sum_bias_ = imu_bias * 100;
    u_bias_fifo_ = std::deque<Eigen::Vector<double, TConfig::INPUT_VECTOR_SIZE>>(100, imu_bias);
  }

  /**
   * @brief returns the current sensor bias
   *
   * @param[out]                  - Eigen::Vector:
   *                                vector containing biases for all IMU measurements
   */
  const Eigen::Vector<double, TConfig::INPUT_VECTOR_SIZE> get_sensor_bias(void) const { return imu_bias_; }

  /**
   * @brief returns a pointer to the param manager
   *
   * @param[out]                  - tam::pmg::ParamValueManager::SharedPtr
   */
  tam::pmg::ParamValueManager::SharedPtr get_param_handler(void) { return param_manager_; }

private:
  /**
   * @brief Update the raw input vector based on the imu measurements
   *
   * @param[in] u_raw             - Eigen::Vector:
   *                                vector containing the raw IMU measurements
   * @param[in] u_fusion_vec      - Eigen::Vector:
   *                                vector indicating whether a IMU measurement is valid
   * @param[out] u                - Eigen::Vector:
   *                                Processed input vector to the Kalman Filter
   *                                [dPsi_radps, ax_mps2, ay_mps2]
   */
  // clang-format off
  const Eigen::Vector<double, TConfig::INPUT_VECTOR_SIZE> average_input_vector(
    const Eigen::Ref<const Eigen::Vector<double, TConfig::INPUT_VECTOR_SIZE*(TConfig::NUM_IMU_MEASUREMENT + TConfig::NUM_BACKUP_IMU_MEASUREMENT)>>& u_raw, // NOLINT
    const Eigen::Ref<const Eigen::Vector<double, TConfig::INPUT_VECTOR_SIZE*(TConfig::NUM_IMU_MEASUREMENT + TConfig::NUM_BACKUP_IMU_MEASUREMENT)>>& u_fusion_vec, // NOLINT
    const Eigen::Ref<const Eigen::Vector<double, TConfig::INPUT_VECTOR_SIZE*(TConfig::NUM_IMU_MEASUREMENT + TConfig::NUM_BACKUP_IMU_MEASUREMENT)>>& u_cov) // NOLINT
  // clang-format on
  {
    // Configurable constants
    constexpr double EPSILON = 1e-6;  // Minimum allowed covariance
    constexpr double MAX_COV = 10.0;  // Maximum allowed covariance

    // clang-format off
    Eigen::Vector<double, TConfig::INPUT_VECTOR_SIZE> u_sum = Eigen::Vector<double, TConfig::INPUT_VECTOR_SIZE>::Zero(); // NOLINT
    Eigen::Vector<double, TConfig::INPUT_VECTOR_SIZE> weight_sum = Eigen::Vector<double, TConfig::INPUT_VECTOR_SIZE>::Zero(); // NOLINT
    Eigen::Vector<double, TConfig::INPUT_VECTOR_SIZE> u = Eigen::Vector<double, TConfig::INPUT_VECTOR_SIZE>::Zero(); // NOLINT
    // clang-format on

    for (int i = 0;
      i < TConfig::INPUT_VECTOR_SIZE * (TConfig::NUM_IMU_MEASUREMENT + TConfig::NUM_BACKUP_IMU_MEASUREMENT); ++i) {
      if (u_fusion_vec[i] > 0.0) {  // Valid measurement
        // Clamp covariance between EPSILON and MAX_COV
        const double clamped_cov = std::min(std::max(u_cov[i], EPSILON), MAX_COV);
        const double weight = 1.0 / clamped_cov;
        u_sum[i % TConfig::INPUT_VECTOR_SIZE] += u_raw[i] * weight;
        weight_sum[i % TConfig::INPUT_VECTOR_SIZE] += weight;
      }
    }

    // take the average of the IMU measurements if a valid measurement was received
    for (int i = 0; i < TConfig::INPUT_VECTOR_SIZE; ++i) {
      if (weight_sum[i] > 0.0) u[i] = u_sum[i] / weight_sum[i];
    }

    return u;
  }

  /**
   * @brief Declare and update the parameters of the IMU Handler
   */
  void declare_and_update_parameters(void)
  {
    imu_handler_params_.filter_coefficients.resize(TConfig::NUM_IMU_MEASUREMENT + TConfig::NUM_BACKUP_IMU_MEASUREMENT);
    for (int i = 0; i < TConfig::NUM_IMU_MEASUREMENT + TConfig::NUM_BACKUP_IMU_MEASUREMENT; ++i) {
      // clang-format off
      const std::vector<double> param_value = param_manager_->declare_and_get_value("imu_" + std::to_string(i + 1) + "_filter_coefficients", std::vector<double>{1.0}, tam::pmg::ParameterType::DOUBLE_ARRAY, "FIR filter coefficients for the IMU").as_double_array(); // NOLINT
      // clang-format on
      imu_handler_params_.filter_coefficients[i] = param_value;
    }
    previous_param_state_hash_ = param_manager_->get_state_hash();
  }

  /**
   * @brief TAM Parameter Manager
   */
  tam::pmg::ParamValueManager::SharedPtr param_manager_ = std::make_shared<tam::pmg::ParamValueManager>();

  /**
   * @brief Variable containing the previous parameter state hash
   */
  std::size_t previous_param_state_hash_{0};

  /**
   * @brief Struct containing the parameters for the IMU Handler
   */
  tam::types::state::config::imu_handler imu_handler_params_{};

  /**
   * @brief Vector containing all the sensor biases for the imu signals
   */
  Eigen::Vector<double, TConfig::INPUT_VECTOR_SIZE> imu_bias_{};

  /**
   * @brief Vector containing all the initally set sensor biases for the imu signals
   */
  Eigen::Vector<double, TConfig::INPUT_VECTOR_SIZE> initial_imu_bias_{};

  /**
   * @brief FiFo containing the last N sensor biases for the imu signals
   */
  std::deque<Eigen::Vector<double, TConfig::INPUT_VECTOR_SIZE>> u_bias_fifo_;

  /**
   * @brief Vector containing the sum of all elements in the fifo
   */
  Eigen::Vector<double, TConfig::INPUT_VECTOR_SIZE> sum_bias_{};

  /**
   * @brief Variable to indicate if the bias should be updated this cycle
   */
  bool update_bias_{false};

  /**
   * @brief External orientation prior [banking, slope, 0.0]
   */
  tam::types::common::Vector3D<double> external_orientation_{};

  /**
   * @brief FIR filters for the imu measurements
   */
  // clang-format off
  std::array<std::unique_ptr<tam::core::state::FIR>, TConfig::INPUT_VECTOR_SIZE*(TConfig::NUM_IMU_MEASUREMENT + TConfig::NUM_BACKUP_IMU_MEASUREMENT)> filter_imu_{};  // NOLINT
  bool initialize_filter_{true};
  // clang-format on
};
}  // namespace tam::core::state
