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
EKF<tam::core::state::EKF_3D>::EKF()
{
  using TConfig = tam::core::state::EKF_3D;
  // ensure that all matrices are set to zero
  A_.setZero();
  B_.setZero();

  // Initialize EKF in 3D
  // set the H_full matrix (this matrix would fuse all measurements)
  // first set the rows that correspond to the localization measurements
  // clang-format off
  for (int i = 0; i < TConfig::NUM_POS_MEASUREMENT; ++i) {
    H_full_.row(i * TConfig::POS_MEASUREMENT_VECTOR_SIZE + TConfig::MEASUREMENT_POS_X_M) << 1, 0, 0, 0, 0, 0, 0, 0, 0; // NOLINT
    H_full_.row(i * TConfig::POS_MEASUREMENT_VECTOR_SIZE + TConfig::MEASUREMENT_POS_Y_M) << 0, 1, 0, 0, 0, 0, 0, 0, 0; // NOLINT
    H_full_.row(i * TConfig::POS_MEASUREMENT_VECTOR_SIZE + TConfig::MEASUREMENT_POS_Z_M) << 0, 0, 1, 0, 0, 0, 0, 0, 0; // NOLINT
  }
  // clang-format on

  // in the next step we set the rows that correspond to the orientation measurements
  // clang-format off
  for (int i = 0; i < TConfig::NUM_ORIENTATION_MEASUREMENT; ++i) {
    H_full_.row(i * TConfig::ORIENTATION_MEASUREMENT_VECTOR_SIZE + TConfig::MEASUREMENT_VECTOR_OFFSET_ORIENTATION + TConfig::MEASUREMENT_PHI_RAD) << 0, 0, 0, 1, 0, 0, 0, 0, 0; // NOLINT
    H_full_.row(i * TConfig::ORIENTATION_MEASUREMENT_VECTOR_SIZE + TConfig::MEASUREMENT_VECTOR_OFFSET_ORIENTATION + TConfig::MEASUREMENT_THETA_RAD) << 0, 0, 0, 0, 1, 0, 0, 0, 0; // NOLINT
    H_full_.row(i * TConfig::ORIENTATION_MEASUREMENT_VECTOR_SIZE + TConfig::MEASUREMENT_VECTOR_OFFSET_ORIENTATION + TConfig::MEASUREMENT_PSI_RAD) << 0, 0, 0, 0, 0, 1, 0, 0, 0; // NOLINT
  }
  // clang-format on

  // in the next step we set the rows that correspond to the velocity measurements
  // clang-format off
  for (int i = 0; i < TConfig::NUM_VEL_MEASUREMENT; ++i) {
    H_full_.row(i * TConfig::VEL_MEASUREMENT_VECTOR_SIZE + TConfig::MEASUREMENT_VECTOR_OFFSET_VEL + TConfig::MEASUREMENT_VX_MPS) << 0, 0, 0, 0, 0, 0, 1, 0, 0; // NOLINT
    H_full_.row(i * TConfig::VEL_MEASUREMENT_VECTOR_SIZE + TConfig::MEASUREMENT_VECTOR_OFFSET_VEL + TConfig::MEASUREMENT_VY_MPS) << 0, 0, 0, 0, 0, 0, 0, 1, 0; // NOLINT
    H_full_.row(i * TConfig::VEL_MEASUREMENT_VECTOR_SIZE + TConfig::MEASUREMENT_VECTOR_OFFSET_VEL + TConfig::MEASUREMENT_VZ_MPS) << 0, 0, 0, 0, 0, 0, 0, 0, 1; // NOLINT
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
void EKF<tam::core::state::EKF_3D>::predict(
  const Eigen::Ref<const Eigen::Vector<double, tam::core::state::EKF_3D::INPUT_VECTOR_SIZE>>& u)
{
  using TConfig = tam::core::state::EKF_3D;
  if (param_manager_->get_state_hash() != previous_param_state_hash_) {
    this->declare_and_update_parameters();
  }

  // Prediction Step of the 3D EKF
  // define the following functions for better readability
  // previous state vector
  const double phi_rad = x_[TConfig::STATE_PHI_RAD];
  const double theta_rad = x_[TConfig::STATE_THETA_RAD];
  const double psi_rad = x_[TConfig::STATE_PSI_RAD];
  const double vx_mps = x_[TConfig::STATE_VX_MPS];
  const double vy_mps = x_[TConfig::STATE_VY_MPS];
  const double vz_mps = x_[TConfig::STATE_VZ_MPS];

  // new input vector
  const double dPhi_radps = u[TConfig::INPUT_DPHI_RADPS];
  const double dTheta_radps = u[TConfig::INPUT_DTHETA_RADPS];
  const double dPsi_radps = u[TConfig::INPUT_DPSI_RADPS];
  const double ax_mps2 = u[TConfig::INPUT_AX_MPS2];
  const double ay_mps2 = u[TConfig::INPUT_AY_MPS2];
  const double az_mps2 = u[TConfig::INPUT_AZ_MPS2];

  // precalculate sin and cos for all angles to increase efficiency
  const double sphi = std::sin(phi_rad), cphi = std::cos(phi_rad);
  const double stheta = std::sin(theta_rad), ctheta = std::cos(theta_rad);
  // guard the Euler angular-rate transform against the theta = +-90 deg gimbal-lock singularity
  // (1 / cos(theta) and tan(theta) diverge there); ctheta_safe equals ctheta away from the pole
  const double ctheta_safe = std::abs(ctheta) < 1e-9 ? std::copysign(1e-9, ctheta) : ctheta;
  const double ttheta = stheta / ctheta_safe;
  const double spsi = std::sin(psi_rad), cpsi = std::cos(psi_rad);

  // State Transition:
  // Rigid body model using accelerations and angular velocities as inputs
  // and applying simple forward integration in vehicle coordinate frame to obtain the velocities.
  // The latter are then used to apply forward integration in global cartesian coordinates to obtain
  // the position. Using the ENU convention (0 degrees heading is east (x-axis)).

  // rotate velocities from the vehicle frame into the enu frame to integrate over them
  const double dx_pos_x = ctheta * cpsi * vx_mps + (sphi * stheta * cpsi - cphi * spsi) * vy_mps +
    (cphi * stheta * cpsi + sphi * spsi) * vz_mps;
  const double dx_pos_y = ctheta * spsi * vx_mps + (sphi * stheta * spsi + cphi * cpsi) * vy_mps +
    (cphi * stheta * spsi - sphi * cpsi) * vz_mps;
  const double dx_pos_z = -stheta * vx_mps + sphi * ctheta * vy_mps + cphi * ctheta * vz_mps;

  // To integrate over the angluar velocity we have to rotate the data into the enu frame
  // https://rotations.berkeley.edu/strapdown-inertial-navigation/
  const double dx_phi = dPhi_radps + sphi * ttheta * dTheta_radps + cphi * ttheta * dPsi_radps;
  const double dx_theta = cphi * dTheta_radps - sphi * dPsi_radps;
  const double dx_psi = sphi / ctheta_safe * dTheta_radps + cphi / ctheta_safe * dPsi_radps;

  // compensate gravity and centrifual accelerations for the imu measurements to get velocities
  // https://acl.kaist.ac.kr/wp-content/uploads/2021/10/2013partd_OJW.pdf (equation 33)
  // |v_x'|     |omega_x|     |v_x|   |a_meas_x|       |    stheata   |
  // |v_y'| = - |omega_y|   x |v_y| + |a_meas_y| + g x |-sphi x ctheta|
  // |v_z'|     |omega_z|_x   |v_z|   |a_meas_z|       |-cphi x ctheta|
  const double dx_vx = ax_mps2 + dPsi_radps * vy_mps - dTheta_radps * vz_mps + stheta * tam::constants::g_earth;
  const double dx_vy = ay_mps2 - dPsi_radps * vx_mps + dPhi_radps * vz_mps - sphi * ctheta * tam::constants::g_earth;
  const double dx_vz = az_mps2 + dTheta_radps * vx_mps - dPhi_radps * vy_mps - cphi * ctheta * tam::constants::g_earth;

  // clang-format off
  Eigen::Vector<double, TConfig::STATE_VECTOR_SIZE> dx;
  dx << dx_pos_x, // NOLINT
        dx_pos_y, // NOLINT
        dx_pos_z, // NOLINT
        dx_phi, // NOLINT
        dx_theta, // NOLINT
        dx_psi, // NOLINT
        dx_vx, // NOLINT
        dx_vy, // NOLINT
        dx_vz; // NOLINT
  // clang-format on

  // partial derivatives of Pos_x_m
  const double dx_pos_x_wrt_phi =
    (cphi * stheta * cpsi + sphi * spsi) * vy_mps + (-sphi * stheta * cpsi + cphi * spsi) * vz_mps;
  const double dx_pos_x_wrt_theta = -stheta * cpsi * vx_mps + sphi * ctheta * cpsi * vy_mps + cphi * ctheta * cpsi * vz_mps;
  const double dx_pos_x_wrt_psi = -ctheta * spsi * vx_mps + (-sphi * stheta * spsi - cphi * cpsi) * vy_mps +
    (-cphi * stheta * spsi + sphi * cpsi) * vz_mps;
  const double dx_pos_x_wrt_vx = ctheta * cpsi;
  const double dx_pos_x_wrt_vy = sphi * stheta * cpsi - cphi * spsi;
  const double dx_pos_x_wrt_vz = cphi * stheta * cpsi + sphi * spsi;

  // partial derivatives of Pos_y_m
  const double dx_pos_y_wrt_phi =
    (cphi * stheta * spsi - sphi * cpsi) * vy_mps + (-sphi * stheta * spsi - cphi * cpsi) * vz_mps;
  const double dx_pos_y_wrt_theta = -stheta * spsi * vx_mps + sphi * ctheta * vy_mps + cphi * ctheta * spsi * vz_mps;
  const double dx_pos_y_wrt_psi = ctheta * cpsi * vx_mps + (sphi * stheta * cpsi - cphi * spsi) * vy_mps +
    (cphi * stheta * cpsi + sphi * spsi) * vz_mps;
  const double dx_pos_y_wrt_vx = ctheta * spsi;
  const double dx_pos_y_wrt_vy = sphi * stheta * spsi + cphi * cpsi;
  const double dx_pos_y_wrt_vz = cphi * stheta * spsi - sphi * cpsi;

  // partial derivatives of Pos_z_m
  const double dx_pos_z_wrt_phi = cphi * ctheta * vy_mps - sphi * ctheta * vz_mps;
  const double dx_pos_z_wrt_theta = -ctheta * vx_mps - sphi * stheta * vy_mps - cphi * stheta * vz_mps;
  const double dx_pos_z_wrt_psi = 0.0;
  const double dx_pos_z_wrt_vx = -stheta;
  const double dx_pos_z_wrt_vy = sphi * ctheta;
  const double dx_pos_z_wrt_vz = cphi * ctheta;

  // partial derivatives of Phi_rad
  const double dx_phi_wrt_phi = cphi * ttheta * dTheta_radps - sphi * ttheta * dPsi_radps;
  const double dx_phi_wrt_theta = (sphi * dTheta_radps + cphi * dPsi_radps) / (ctheta_safe * ctheta_safe);
  const double dx_phi_wrt_psi = 0.0;

  // partial derivatives of Theta_rad
  const double dx_theta_wrt_phi = -sphi * dTheta_radps - cphi * dPsi_radps;
  const double dx_theta_wrt_theta = 0.0;
  const double dx_theta_wrt_psi = 0.0;

  // partial derivatives of Psi_rad
  const double dx_psi_wrt_phi = cphi / ctheta_safe * dTheta_radps - sphi / ctheta_safe * dPsi_radps;
  const double dx_psi_wrt_theta = (sphi * dTheta_radps + cphi * dPsi_radps) * stheta / ctheta_safe / ctheta_safe;
  const double dx_psi_wrt_psi = 0.0;

  // partial derivatives of vx_mps
  const double dx_vx_wrt_vx = 0.0;
  const double dx_vx_wrt_vy = dPsi_radps;
  const double dx_vx_wrt_vz = -dTheta_radps;

  // partial derivatives of vy_mps
  const double dx_vy_wrt_vx = -dPsi_radps;
  const double dx_vy_wrt_vy = 0.0;
  const double dx_vy_wrt_vz = dPhi_radps;

  // partial derivatives of vz_mps
  const double dx_vz_wrt_vx = dTheta_radps;
  const double dx_vz_wrt_vy = -dPhi_radps;
  const double dx_vz_wrt_vz = 0.0;

  // construct the Jacobian matrix consisting of partial derivatives with respect to the
  // system state from the system update equation
  // clang-format off
  A_ << 0, 0, 0, dx_pos_x_wrt_phi, dx_pos_x_wrt_theta, dx_pos_x_wrt_psi, dx_pos_x_wrt_vx, dx_pos_x_wrt_vy, dx_pos_x_wrt_vz, // NOLINT
        0, 0, 0, dx_pos_y_wrt_phi, dx_pos_y_wrt_theta, dx_pos_y_wrt_psi, dx_pos_y_wrt_vx, dx_pos_y_wrt_vy, dx_pos_y_wrt_vz, // NOLINT
        0, 0, 0, dx_pos_z_wrt_phi, dx_pos_z_wrt_theta, dx_pos_z_wrt_psi, dx_pos_z_wrt_vx, dx_pos_z_wrt_vy, dx_pos_z_wrt_vz, // NOLINT
        0, 0, 0, dx_phi_wrt_phi,   dx_phi_wrt_theta,   dx_phi_wrt_psi,   0, 0, 0, // NOLINT
        0, 0, 0, dx_theta_wrt_phi, dx_theta_wrt_theta, dx_theta_wrt_psi, 0, 0, 0, // NOLINT
        0, 0, 0, dx_psi_wrt_phi,   dx_psi_wrt_theta,   dx_psi_wrt_psi,   0, 0, 0, // NOLINT
        0, 0, 0, 0, 0, 0, dx_vx_wrt_vx, dx_vx_wrt_vy, dx_vx_wrt_vz, // NOLINT
        0, 0, 0, 0, 0, 0, dx_vy_wrt_vx, dx_vy_wrt_vy, dx_vy_wrt_vz, // NOLINT
        0, 0, 0, 0, 0, 0, dx_vz_wrt_vx, dx_vz_wrt_vy, dx_vz_wrt_vz; // NOLINT
  // clang-format on

  A_ = Eigen::Matrix<double, TConfig::STATE_VECTOR_SIZE, TConfig::STATE_VECTOR_SIZE>::Identity() + A_ * TConfig::TS;

  // Jacobian matrix B consisting of partial derivatives with respect to the
  // system input from the system update equation
  // partial derivatives of Phi_rad
  const double dx_phi_wrt_dphi = 1.0;
  const double dx_phi_wrt_dtheta = sphi * ttheta;
  const double dx_phi_wrt_dpsi = cphi * ttheta;

  // partial derivatives of Theta_rad
  const double dx_theta_wrt_dphi = 0.0;
  const double dx_theta_wrt_dtheta = cphi;
  const double dx_theta_wrt_dpsi = -sphi;
  // partial derivatives of Psi_rad
  const double dx_psi_wrt_dphi = 0.0;
  const double dx_psi_wrt_dtheta = sphi / ctheta_safe;
  const double dx_psi_wrt_dpsi = cphi / ctheta_safe;

  // partial derivatives of vx_mps
  const double dx_vx_wrt_dphi = 0.0;
  const double dx_vx_wrt_dtheta = -vz_mps;
  const double dx_vx_wrt_dpsi = vy_mps;

  // partial derivatives of vy_mps
  const double dx_vy_wrt_dphi = vz_mps;
  const double dx_vy_wrt_dtheta = 0.0;
  const double dx_vy_wrt_dpsi = -vx_mps;

  // partial derivatives of vz_mps
  const double dx_vz_wrt_dphi = -vy_mps;
  const double dx_vz_wrt_dtheta = vx_mps;
  const double dx_vz_wrt_dpsi = 0.0;

  // clang-format off
  B_ << 0,                 0,                   0,                 0, 0, 0, // NOLINT
        0,                 0,                   0,                 0, 0, 0, // NOLINT
        0,                 0,                   0,                 0, 0, 0, // NOLINT
        dx_phi_wrt_dphi,   dx_phi_wrt_dtheta,   dx_phi_wrt_dpsi,   0, 0, 0, // NOLINT
        dx_theta_wrt_dphi, dx_theta_wrt_dtheta, dx_theta_wrt_dpsi, 0, 0, 0, // NOLINT
        dx_psi_wrt_dphi,   dx_psi_wrt_dtheta,   dx_psi_wrt_dpsi,   0, 0, 0, // NOLINT
        dx_vx_wrt_dphi,    dx_vx_wrt_dtheta,    dx_vx_wrt_dpsi,    1, 0, 0, // NOLINT
        dx_vy_wrt_dphi,    dx_vy_wrt_dtheta,    dx_vy_wrt_dpsi,    0, 1, 0, // NOLINT
        dx_vz_wrt_dphi,    dx_vz_wrt_dtheta,    dx_vz_wrt_dpsi,    0, 0, 1; // NOLINT
  // clang-format on

  B_ = TConfig::TS * B_;

  // fuse the process noise covariance matrix with
  // the linearized version of the system input vector (non-additive noise)
  const Eigen::Matrix<double, TConfig::STATE_VECTOR_SIZE, TConfig::STATE_VECTOR_SIZE> Q_lin = B_ * Q_ * B_.transpose();

  // use euler integration to integrate over one timestep
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
EKF<tam::core::state::EKF_3D>::get_debug(void)
{
  using TConfig = tam::core::state::EKF_3D;
  // get the debug outputs of the 3D EKF
  // Iterate through all localization residuals
  for (int i = 0; i < TConfig::NUM_POS_MEASUREMENT; ++i) {
    // Get a reference to the vector for this ID once
    auto& debug = debug_output_[{tam::types::state::measurements::POS, static_cast<uint8_t>(i)}];
    debug.clear();

    // cartesian position residuals in meters
    const uint8_t idx_pos_x = i * TConfig::POS_MEASUREMENT_VECTOR_SIZE + TConfig::MEASUREMENT_POS_X_M;
    const uint8_t idx_pos_y = i * TConfig::POS_MEASUREMENT_VECTOR_SIZE + TConfig::MEASUREMENT_POS_Y_M;
    const uint8_t idx_pos_z = i * TConfig::POS_MEASUREMENT_VECTOR_SIZE + TConfig::MEASUREMENT_POS_Z_M;
    debug.emplace_back("res_x_m", residuals_raw_[idx_pos_x]);
    debug.emplace_back("res_y_m", residuals_raw_[idx_pos_y]);
    debug.emplace_back("res_z_m", residuals_raw_[idx_pos_z]);

    // cartesian position measurement variance
    debug.emplace_back("R_x_m", R_(idx_pos_x, idx_pos_x));
    debug.emplace_back("R_y_m", R_(idx_pos_y, idx_pos_y));
    debug.emplace_back("R_z_m", R_(idx_pos_z, idx_pos_z));

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

    // cartesian orientation residuals in rad
    // clang-format off
    const uint8_t idx_phi = TConfig::MEASUREMENT_VECTOR_OFFSET_ORIENTATION + i * TConfig::ORIENTATION_MEASUREMENT_VECTOR_SIZE + TConfig::MEASUREMENT_PHI_RAD; // NOLINT
    const uint8_t idx_theta = TConfig::MEASUREMENT_VECTOR_OFFSET_ORIENTATION + i * TConfig::ORIENTATION_MEASUREMENT_VECTOR_SIZE + TConfig::MEASUREMENT_THETA_RAD; // NOLINT
    const uint8_t idx_psi = TConfig::MEASUREMENT_VECTOR_OFFSET_ORIENTATION + i * TConfig::ORIENTATION_MEASUREMENT_VECTOR_SIZE + TConfig::MEASUREMENT_PSI_RAD; // NOLINT
    debug.emplace_back("res_phi_rad", residuals_raw_[idx_phi]);
    debug.emplace_back("res_theta_rad", residuals_raw_[idx_theta]);
    debug.emplace_back("res_psi_rad", residuals_raw_[idx_psi]);
    // clang-format on

    // cartesian orientation measurement variance
    debug.emplace_back("R_phi_rad", R_(idx_phi, idx_phi));
    debug.emplace_back("R_theta_rad", R_(idx_theta, idx_theta));
    debug.emplace_back("R_psi_rad", R_(idx_psi, idx_psi));
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
    const uint8_t idx_pos_vz = TConfig::MEASUREMENT_VECTOR_OFFSET_VEL + i * TConfig::VEL_MEASUREMENT_VECTOR_SIZE + TConfig::MEASUREMENT_VZ_MPS; // NOLINT
    debug.emplace_back("res_vx_mps", residuals_raw_[idx_pos_vx]);
    debug.emplace_back("res_vy_mps", residuals_raw_[idx_pos_vy]);
    debug.emplace_back("res_vz_mps", residuals_raw_[idx_pos_vz]);
    // clang-format on
  }
  return debug_output_;
}
}  // namespace tam::core::state
