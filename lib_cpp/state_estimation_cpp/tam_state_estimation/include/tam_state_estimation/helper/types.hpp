/*
 * Copyright 2026 Marcel Weinmann, Maximilian Leitenstern
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
#include <string>
#include <vector>

// type definitions
#include "tum_types_cpp/common.hpp"
#include "tum_types_cpp/control.hpp"

// State Estimation Base Class
#include "state_estimation_base/state_estimation_base.hpp"

namespace tam::types::state::config {
/**
 * @brief Struct containing the parameters for the state estimation class
 */
struct state_estimation {
  bool initialize_from_measurements{};
  bool fuse_external_orientation{};
  bool fuse_ref_angles{};
  std::vector<double> external_orientation_R_init{};
  std::vector<double> external_orientation_outlier_bounds{};
  double overwrite_pose_covariance_threshold{};
  double hard_linear_velocity_outlier_th{};
  double hard_accelerometer_outlier_th{};
  double hard_angular_velocity_outlier_th{};
  int max_consecutive_vel_hard_outliers{};
  int max_consecutive_imu_hard_outliers{};
  std::vector<double> u_bias_init{};
};

/**
 * @brief Struct containing the parameters for the kalman filter
 */
struct kalman_filter {
  bool enable_measurement_covariance_adaptation{};
  bool enable_input_cross_correlation{};
  bool enable_mahalanobis_outlier_rejection{};
  double covariance_adaption_limit{};
  double covariance_adaption_decay{};
  std::vector<double> R_init{};
  std::vector<double> Q{};
  std::vector<double> outlier_bounds{};
};

/**
 * @brief Struct containing the parameters for the Reference Handler
 */
struct ref_orientation_handler {
  std::vector<double> v_dot_filter_coefficients{};
  std::vector<double> R_init{};
  std::vector<double> outlier_bounds{};
};

/**
 * @brief Struct containing the parameters for the State Machine
 */
struct state_machine {
  int min_valid_imus{};
};

/**
 * @brief Struct containing the parameters for the vehicle model
 */
struct vehicle_model {
  double tyre_radius_front_m{};
  double tyre_radius_rear_m{};
  double l_WheelbaseF_m{};
  double l_WheelbaseR_m{};
  std::vector<double> rr_vel_scale_vel_points_mps{};
  std::vector<double> rr_vel_scale_scale_factors{};
  std::vector<double> R_init{};
  std::vector<double> outlier_bounds{};
};

/**
 * @brief Struct containing the parameters for the IMU Handler
 */
struct imu_handler {
  std::vector<std::vector<double>> filter_coefficients{};
};
}  // namespace tam::types::state::config

namespace tam::core::state {
/**
 * @brief Vehicle model used as a compile-time template argument:
 *        - Kinematic:    the wheelspeed odometry is not fused
 *        - NonHolonomic: the wheelspeed odometry is fused as a linear velocity
 *                        measurement (vy = 0)
 *        - SingleTrack:  the wheelspeed odometry is fused and the steering angle
 *                        is used to estimate a side slip angle (linear single
 *                        track model)
 */
enum class VehicleModel { Kinematic, NonHolonomic, SingleTrack };
}  // namespace tam::core::state
