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
#include "tam_state_estimation/kalman_filter/ekf.hpp"

namespace tam::core::state {
template <>
EKF<tam::core::state::EKF_2D>::EKF()
{
  using TConfig = tam::core::state::EKF_2D;
  // ensure that all matrices are set to zero
  A_.setZero();
  B_.setZero();

  // Initialize EKF in 2D
  // set the H_full matrix (this matrix would fuse all measurements)
  // first set the rows that correspond to the localization measurements
  // clang-format off
  for (int i = 0; i < TConfig::NUM_POS_MEASUREMENT; ++i) {
    H_full_.row(i * TConfig::POS_MEASUREMENT_VECTOR_SIZE + TConfig::MEASUREMENT_POS_X_M) << 1, 0, 0, 0, 0; // NOLINT
    H_full_.row(i * TConfig::POS_MEASUREMENT_VECTOR_SIZE + TConfig::MEASUREMENT_POS_Y_M) << 0, 1, 0, 0, 0; // NOLINT
  }
  // clang-format on

  // in the next step we set the rows that correspond to the orientation measurements
  // clang-format off
  for (int i = 0; i < TConfig::NUM_ORIENTATION_MEASUREMENT; ++i) {
    H_full_.row(i * TConfig::ORIENTATION_MEASUREMENT_VECTOR_SIZE + TConfig::MEASUREMENT_VECTOR_OFFSET_ORIENTATION + TConfig::MEASUREMENT_PSI_RAD) << 0, 0, 1, 0, 0; // NOLINT
  }
  // clang-format on

  // in the next step we set the rows that correspond to the velocity measurements
  // clang-format off
  for (int i = 0; i < TConfig::NUM_VEL_MEASUREMENT; ++i) {
    H_full_.row(i * TConfig::VEL_MEASUREMENT_VECTOR_SIZE + TConfig::MEASUREMENT_VECTOR_OFFSET_VEL + TConfig::MEASUREMENT_VX_MPS) << 0, 0, 0, 1, 0; // NOLINT
    H_full_.row(i * TConfig::VEL_MEASUREMENT_VECTOR_SIZE + TConfig::MEASUREMENT_VECTOR_OFFSET_VEL + TConfig::MEASUREMENT_VY_MPS) << 0, 0, 0, 0, 1; // NOLINT
  }
  // clang-format on

  this->declare_and_update_parameters();
}

/**
 * @brief Prediction step of the EKF
 *
 * @param[in] u             - Input vector:
 *                            [dPsi_radps, ax_mps2, ay_mps2]
 */
template <>
void EKF<tam::core::state::EKF_2D>::predict(
  const Eigen::Ref<const Eigen::Vector<double, tam::core::state::EKF_2D::INPUT_VECTOR_SIZE>>& u)
{
  using TConfig = tam::core::state::EKF_2D;
  if (param_manager_->get_state_hash() != previous_param_state_hash_) {
    this->declare_and_update_parameters();
  }

  // Prediction Step of the 2D EKF
  // define the following functions for better readability
  // previous state vector
  const double psi_rad = x_[TConfig::STATE_PSI_RAD];
  const double vx_mps = x_[TConfig::STATE_VX_MPS];
  const double vy_mps = x_[TConfig::STATE_VY_MPS];

  // new input vector
  const double dPsi_radps = u[TConfig::INPUT_DPSI_RADPS];
  const double ax_mps2 = u[TConfig::INPUT_AX_MPS2];
  const double ay_mps2 = u[TConfig::INPUT_AY_MPS2];

  // construct the Jacobian matrix consisting of partial derivatives with respect to the
  // system state from the system update equation
  // clang-format off
  A_ << 0, 0, (-std::sin(psi_rad) * vx_mps - std::cos(psi_rad) * vy_mps), std::cos(psi_rad), -std::sin(psi_rad), // NOLINT
        0, 0, (std::cos(psi_rad) * vx_mps - std::sin(psi_rad) * vy_mps),  std::sin(psi_rad), std::cos(psi_rad), // NOLINT
        0, 0, 0, 0, 0, // NOLINT
        0, 0, 0, 0, dPsi_radps, // NOLINT
        0, 0, 0, -dPsi_radps, 0; // NOLINT
  // clang-format on

  A_ = Eigen::Matrix<double, TConfig::STATE_VECTOR_SIZE, TConfig::STATE_VECTOR_SIZE>::Identity() + A_ * TConfig::TS;

  // Jacobian matrix B consisting of partial derivatives with respect to the
  // system input from the system update equation
  // clang-format off
  B_ << 0, 0, 0, // NOLINT
        0, 0, 0, // NOLINT
        1, 0, 0, // NOLINT
        vy_mps, 1, 0, // NOLINT
        -vx_mps, 0, 1; // NOLINT
  // clang-format on

  B_ = TConfig::TS * B_;

  // fuse the process noise covariance matrix with
  // the linearized version of the system input vector (non-additive noise)
  const Eigen::Matrix<double, TConfig::STATE_VECTOR_SIZE, TConfig::STATE_VECTOR_SIZE> Q_lin = B_ * Q_ * B_.transpose();

  // State Transition:
  // Rigid body model using accelerations and yaw rate as inputs and applying simple forward
  // integration in vehicle coordinate frame to obtain the velocities. The latter are then
  // used to apply forward integration in global cartesian coordinates to obtain the
  // position. Using the ENU convention (0 degrees heading is east (x-axis)).
  Eigen::Vector<double, TConfig::STATE_VECTOR_SIZE> dx;
  // clang-format off
  dx << std::cos(psi_rad) * vx_mps - std::sin(psi_rad) * vy_mps, // NOLINT
        std::sin(psi_rad) * vx_mps + std::cos(psi_rad) * vy_mps, // NOLINT
        dPsi_radps, // NOLINT
        ax_mps2 + dPsi_radps * vy_mps, // NOLINT
        ay_mps2 - dPsi_radps * vx_mps; // NOLINT
  // clang-format on

  // integrate over on timestep
  x_pred_ = x_ + TConfig::TS * dx;

  // Normalize the angles of every angle in the state transition vector
  for (int i = 0; i < TConfig::STATE_VECTOR_SIZE; i++) {
    if (TConfig::STATE_VECTOR_ANGLE_INDICATOR[i] == true) {
      x_pred_[i] = tam::helpers::geometry::normalize_angle(x_pred_[i]);
    }
  }

  // Predict the state covariance
  P_pred_.noalias() = A_ * P_ * A_.transpose() + Q_lin;
}

/**
 * @brief get all debug values of the kalman filter
 *
 * @param[out]              - std::unordered_map:
 *                            residuals of the Extended Kalman Filter
 */
template <>
const tam::types::state::unordered_identifier_map<std::vector<std::pair<std::string, double>>>&
EKF<tam::core::state::EKF_2D>::get_debug(void)
{
  using TConfig = tam::core::state::EKF_2D;
  // get the debug outputs of the 2D EKF
  // Iterate through all position residuals
  for (int i = 0; i < TConfig::NUM_POS_MEASUREMENT; ++i) {
    // Get a reference to the vector for this ID once
    auto& debug = debug_output_[{tam::types::state::measurements::POS, static_cast<uint8_t>(i)}];
    debug.clear();

    // cartesian position residuals in meters
    const uint8_t idx_pos_x = i * TConfig::POS_MEASUREMENT_VECTOR_SIZE + TConfig::MEASUREMENT_POS_X_M;
    const uint8_t idx_pos_y = i * TConfig::POS_MEASUREMENT_VECTOR_SIZE + TConfig::MEASUREMENT_POS_Y_M;
    debug.emplace_back("res_x_m", residuals_raw_[idx_pos_x]);
    debug.emplace_back("res_y_m", residuals_raw_[idx_pos_y]);

    // cartesian position measurement variance
    debug.emplace_back("R_x_m", R_(idx_pos_x, idx_pos_x));
    debug.emplace_back("R_y_m", R_(idx_pos_y, idx_pos_y));

    // frenet position residuals in meters
    // clang-format off
    debug.emplace_back("res_s_m", std::cos(x_[TConfig::STATE_PSI_RAD]) * residuals_raw_[idx_pos_x] + std::sin(x_[TConfig::STATE_PSI_RAD]) * residuals_raw_[idx_pos_y]); // NOLINT
    debug.emplace_back("res_d_m", -std::sin(x_[TConfig::STATE_PSI_RAD]) * residuals_raw_[idx_pos_x] + std::cos(x_[TConfig::STATE_PSI_RAD]) * residuals_raw_[idx_pos_y]); // NOLINT
    // clang-format on
  }

  // Iterate through all orientation residuals
  for (int i = 0; i < TConfig::NUM_ORIENTATION_MEASUREMENT; ++i) {
    // Get a reference to the vector for this ID once
    auto& debug = debug_output_[{tam::types::state::measurements::ORIENTATION, static_cast<uint8_t>(i)}];
    debug.clear();

    // cartesian orientation residual and measurement in rad
    // clang-format off
    const uint8_t idx_psi = TConfig::MEASUREMENT_VECTOR_OFFSET_ORIENTATION + i * TConfig::ORIENTATION_MEASUREMENT_VECTOR_SIZE + TConfig::MEASUREMENT_PSI_RAD; // NOLINT
    debug.emplace_back("res_psi_rad", residuals_raw_[idx_psi]);
    debug.emplace_back("R_psi", R_(idx_psi, idx_psi));
    // clang-format on
  }

  // Iterate through all linear velocity residuals
  for (int i = 0; i < TConfig::NUM_VEL_MEASUREMENT; ++i) {
    // Get a reference to the vector for this ID once
    auto& debug = debug_output_[{tam::types::state::measurements::VEL, static_cast<uint8_t>(i)}];
    debug.clear();

    // velocity residuals in the vehicle frame in meters
    // clang-format off
    const uint8_t idx_pos_vx = TConfig::MEASUREMENT_VECTOR_OFFSET_VEL + i * TConfig::VEL_MEASUREMENT_VECTOR_SIZE + TConfig::MEASUREMENT_VX_MPS; // NOLINT
    const uint8_t idx_pos_vy = TConfig::MEASUREMENT_VECTOR_OFFSET_VEL + i * TConfig::VEL_MEASUREMENT_VECTOR_SIZE + TConfig::MEASUREMENT_VY_MPS; // NOLINT
    debug.emplace_back("res_vx_mps", residuals_raw_[idx_pos_vx]);
    debug.emplace_back("res_vy_mps", residuals_raw_[idx_pos_vy]);
    // clang-format on
  }
  return debug_output_;
}
}  // namespace tam::core::state
