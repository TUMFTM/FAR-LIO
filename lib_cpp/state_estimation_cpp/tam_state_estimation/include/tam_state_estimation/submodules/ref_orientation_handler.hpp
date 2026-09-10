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
#include <eigen3/Eigen/Dense>
#include <vector>

// type definitions
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
template <typename TConfig>
class RefOrientationHandler
{
public:
  /**
   * @brief Constructor
   */
  RefOrientationHandler() { this->declare_and_update_parameters(); };

  /**
   * @brief Update the reference orientation based on the imu measurements and the vehicle odometry
   *
   * @param[in] x                 - Eigen::Vector:
   *                                output of the last kalman filter step (vehicle state)
   *
   * @param[in] u                 - Eigen::Vector:
   *                                filtered and averaged imu measurements
   *
   * @param[out]                  - tam::types::control::Odometry:
   *                                calculated reference angles
   */
  const tam::types::control::Odometry& update(
    [[maybe_unused]] const Eigen::Ref<const Eigen::Vector<double, TConfig::STATE_VECTOR_SIZE>>& x,
    [[maybe_unused]] const Eigen::Ref<const Eigen::Vector<double, TConfig::INPUT_VECTOR_SIZE>>& u)
    requires(std::is_same_v<TConfig, tam::core::state::EKF_2D>)
  {
    // it does not make sense to compute the reference angles in 2d thats why we just return 0
    return reference_orientation_;
  };

  const tam::types::control::Odometry& update(
    const Eigen::Ref<const Eigen::Vector<double, TConfig::STATE_VECTOR_SIZE>>& x,
    const Eigen::Ref<const Eigen::Vector<double, TConfig::INPUT_VECTOR_SIZE>>& u)
    requires(!std::is_same_v<TConfig, tam::core::state::EKF_2D>)
  {
    if (param_manager_->get_state_hash() != previous_param_state_hash_) {
      this->declare_and_update_parameters();
    }
    // initialize the numerical differentiation
    if (first_iteration_) {
      v_t_minus_two_ = x.segment(TConfig::STATE_VX_MPS, TConfig::VEL_MEASUREMENT_VECTOR_SIZE);

      // set the Filter Coefficients
      const Eigen::VectorXd imu_filter_coefficients =
        Eigen::Map<Eigen::VectorXd>(ref_orientation_handler_params_.v_dot_filter_coefficients.data(),
          ref_orientation_handler_params_.v_dot_filter_coefficients.size());

      for (std::size_t i = 0; i < TConfig::VEL_MEASUREMENT_VECTOR_SIZE; ++i) {
        filter_[i] = std::make_unique<tam::core::state::FIR>(imu_filter_coefficients);
      }

      first_iteration_ = false;
      return reference_orientation_;
    }
    if (second_iteration_) {
      v_t_minus_one_ = x.segment(TConfig::STATE_VX_MPS, TConfig::VEL_MEASUREMENT_VECTOR_SIZE);
      second_iteration_ = false;
      return reference_orientation_;
    }

    // compute the central derivative of the vehicle velocity
    Eigen::Vector<double, TConfig::VEL_MEASUREMENT_VECTOR_SIZE> v_dot{};
    v_dot =
      (x.segment(TConfig::STATE_VX_MPS, TConfig::VEL_MEASUREMENT_VECTOR_SIZE) - v_t_minus_two_) / (2 * TConfig::TS);

    // buffer the vehicle velocities for the next cycle
    v_t_minus_two_ = v_t_minus_one_;
    v_t_minus_one_ = x.segment(TConfig::STATE_VX_MPS, TConfig::VEL_MEASUREMENT_VECTOR_SIZE);

    // FIR Filter the velocity derivative
    for (int i = 0; i < TConfig::VEL_MEASUREMENT_VECTOR_SIZE; ++i) {
      v_dot[i] = filter_[i]->step(v_dot[i]);
    }

    // calculate the reference angles base on:
    // https://acl.kaist.ac.kr/wp-content/uploads/2021/10/2013partd_OJW.pdf
    // to ensure that no inf is returned by the asin the values have to be clamped
    const double phi_value = (-v_dot[TConfig::MEASUREMENT_VY_MPS] + u[TConfig::INPUT_AY_MPS2] -
                         u[TConfig::INPUT_DPSI_RADPS] * x[TConfig::STATE_VX_MPS] +
                         u[TConfig::INPUT_DPHI_RADPS] * x[TConfig::STATE_VZ_MPS]) /
      (tam::constants::g_earth * std::cos(x[TConfig::STATE_THETA_RAD]));

    // clamp bounds the +-inf case; guard the 0/0 = NaN case so asin never propagates a NaN
    reference_orientation_.orientation_rad.x =
      std::asin(std::clamp(std::isnan(phi_value) ? 0.0 : phi_value, -0.5, 0.5));

    const double theta_value = (v_dot[TConfig::MEASUREMENT_VX_MPS] - u[TConfig::INPUT_AX_MPS2] -
                           u[TConfig::INPUT_DPSI_RADPS] * x[TConfig::STATE_VY_MPS] +
                           u[TConfig::INPUT_DTHETA_RADPS] * x[TConfig::STATE_VZ_MPS]) /
      tam::constants::g_earth;

    reference_orientation_.orientation_rad.y =
      std::asin(std::clamp(std::isnan(theta_value) ? 0.0 : theta_value, -0.5, 0.5));

    return reference_orientation_;
  };

  /**
   * @brief Get the current status (decide whether the reference angle should be fused)
   *
   * @param[out]                          - tam::types::ErrorLvl:
   *                                        current fusion status
   */
  tam::types::ErrorLvl get_status(void)
    requires(std::is_same_v<TConfig, tam::core::state::EKF_2D>)
  {
    return tam::types::ErrorLvl::ERROR;
  };

  tam::types::ErrorLvl get_status(void)
    requires(!std::is_same_v<TConfig, tam::core::state::EKF_2D>)
  {
    if (first_iteration_ || second_iteration_) return tam::types::ErrorLvl::ERROR;
    return tam::types::ErrorLvl::OK;
  };

  /**
   * @brief returns a pointer to the param manager
   *
   * @param[out]                  - std::shared_ptr<tam::interfaces::ParamManagerBase>
   */
  tam::pmg::ParamValueManager::SharedPtr get_param_handler(void) { return param_manager_; };

private:
  /**
   * @brief Declare and update the parameters of the Reference orientation handler
   */
  void declare_and_update_parameters(void)
  {
    // clang-format off
    ref_orientation_handler_params_.v_dot_filter_coefficients = param_manager_->declare_and_get_value("reference_angles.v_dot_filter_coefficients", std::vector<double>{1.0}, tam::pmg::ParameterType::DOUBLE_ARRAY, "FIR filter coefficients for the v_dot calculation of the reference angles").as_double_array(); // NOLINT
    ref_orientation_handler_params_.R_init = param_manager_->declare_and_get_value("reference_angles.orientation.R_init", std::vector<double>{0.1, 0.1}, tam::pmg::ParameterType::DOUBLE_ARRAY, "Constant Measurement Noise Covariance for the reference orientations").as_double_array(); // NOLINT
    ref_orientation_handler_params_.outlier_bounds = param_manager_->declare_and_get_value("reference_angles.orientation.outlier_bounds", std::vector<double>{0.1, 0.1}, tam::pmg::ParameterType::DOUBLE_ARRAY, "Constant Outlier Bounds for the reference orientations").as_double_array(); // NOLINT
    // clang-format on
    previous_param_state_hash_ = param_manager_->get_state_hash();
  };

private:
  /**
   * @brief calculated reference orientation
   */
  tam::types::control::Odometry reference_orientation_{};

  /**
   * @brief predicted vehicle velocity from one timesteps ago
   */
  Eigen::Vector<double, TConfig::VEL_MEASUREMENT_VECTOR_SIZE> v_t_minus_one_{};

  /**
   * @brief predicted vehicle velocity from two timesteps ago
   */
  Eigen::Vector<double, TConfig::VEL_MEASUREMENT_VECTOR_SIZE> v_t_minus_two_{};

  /**
   * @brief FIR filters for the numerical derivative of the vehicle velocity
   */
  std::array<std::unique_ptr<tam::core::state::FIR>, TConfig::VEL_MEASUREMENT_VECTOR_SIZE> filter_{};

  /**
   * @brief booleans to properly initialize the numerical differentiation
   */
  bool first_iteration_{true};
  bool second_iteration_{true};

  /**
   * @brief Variable containing the previous parameter state hash
   */
  std::size_t previous_param_state_hash_{0};

  /**
   * @brief Struct containing the parameters for the Reference Handler
   */
  tam::types::state::config::ref_orientation_handler ref_orientation_handler_params_{};

  /**
   * @brief TAM Parameter Manager
   */
  tam::pmg::ParamValueManager::SharedPtr param_manager_ = std::make_shared<tam::pmg::ParamValueManager>();
};
}  // namespace tam::core::state
