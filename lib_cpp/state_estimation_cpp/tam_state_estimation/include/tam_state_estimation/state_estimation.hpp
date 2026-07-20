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

#include <cmath>
#include <eigen3/Eigen/Dense>
#include <iostream>
#include <map>
#include <memory>
#include <string>
#include <vector>

// type definitions
#include "helper/types.hpp"
#include "tum_types_cpp/common.hpp"
#include "tum_types_cpp/control.hpp"

// type convertions
#include "tum_helpers_cpp/rotations.hpp"
#include "tum_type_conversions_ros_cpp/tum_type_conversions.hpp"

// Param manager
#include "param_management_cpp/param_manager_composer.hpp"
#include "param_management_cpp/param_value_manager.hpp"

// State Estimation constants / template input
#include "state_estimation_constants/EKF_2D.hpp"
#include "state_estimation_constants/EKF_3D.hpp"

// State Estimation Base Class
#include "state_estimation_base/state_estimation_base.hpp"

// Kalman Filters
#include "tam_state_estimation/kalman_filter/base.hpp"
#include "tam_state_estimation/kalman_filter/ekf.hpp"

// State Estimation State Machine
#include "tam_state_estimation/submodules/state_machine.hpp"

// Vehicle Model Odometry
#include "tam_state_estimation/submodules/vehicle_model_handler.hpp"

// IMU Preprocessing
#include "tam_state_estimation/submodules/imu_handler.hpp"

// Reference Orientation
#include "tam_state_estimation/submodules/ref_orientation_handler.hpp"

// Outlier detection helper
#include "tam_state_estimation/helper/outlier_rejection.hpp"

// TSL Logger
#include "tsl_logger_cpp/value_logger.hpp"

namespace tam::core::state {
template <typename TConfig, VehicleModel TModel>
class StateEstimation : public StateEstimationBase
{
public:
  /**
   * @brief Constructor
   */
  StateEstimation()
  {
    // initialize all subclasses
    state_machine_ = std::make_unique<tam::core::state::StateMachine<TConfig>>();
    imu_handler_ = std::make_unique<tam::core::state::IMUHandler<TConfig>>();
    ref_orientation_handler_ = std::make_unique<tam::core::state::RefOrientationHandler<TConfig>>();
    vehicle_model_handler_ = std::make_shared<tam::core::state::VehicleModelHandler<TConfig, TModel>>();

    // initialize kalman filter depending on which template is chosen
    kalman_filter_ = std::make_unique<tam::core::state::EKF<TConfig>>();

    // param_manager
    param_manager_composer_ = std::make_shared<tam::pmg::ParamManagerComposer>(
      std::vector<std::shared_ptr<tam::pmg::MgmtInterface>>{param_manager_, state_machine_->get_param_handler(),
        vehicle_model_handler_->get_param_handler(), imu_handler_->get_param_handler(),
        ref_orientation_handler_->get_param_handler(), kalman_filter_->get_param_handler()});
    this->declare_and_update_parameters();

    // set the input and measurent vector to zero
    u_.setZero();
    u_raw_.setZero();
    z_.setZero();
    x_out_.setZero();
    fusion_vec_.setZero();
    outlier_buffer_.setZero();
    updated_acceleration_.setZero();
    updated_angular_velocity_.setZero();
  }

  /**
   * @brief Steps the state estimation once
   */
  void step(void) override
  {
    if (param_manager_->get_state_hash() != previous_param_state_hash_) {
      this->declare_and_update_parameters();
    }
    vehicle_model_handler_->declare_and_update_parameters();

    // preprocess the input vector containing linear accelerations and angular veocities
    // the valid_u_fusion_vec ensures that only valid measurements are fused
    u_ = imu_handler_->update_input_vector(u_raw_, state_machine_->get_valid_u_fusion_vec(), u_cov_);
    // update the reference orientation based on the updated imu measurements
    // and the last predicted vehicle odometry
    if (state_estimation_params_.fuse_ref_angles) {
      // set the output as last orientation measurement
      const tam::types::control::Odometry ref_orientation = ref_orientation_handler_->update(x_out_, u_);
      ref_angles_ = ref_orientation.orientation_rad;
      StateEstimation<TConfig, TModel>::set_input_orientation(
        ref_orientation, TConfig::NUM_ORIENTATION_MEASUREMENT - 1, true, true, false);
      StateEstimation<TConfig, TModel>::set_input_orientation_status(
        ref_orientation_handler_->get_status(), TConfig::NUM_ORIENTATION_MEASUREMENT - 1);
    }

    // update the measurement matrix of the Kalman filter to only fuse updated and valid measurements
    if (kalman_filter_->get_enable_adaptive_measurement_covariance()) {
      // update the measurement covariance matrix of the Kalman filter
      kalman_filter_->update_measurement_covariance_matrix(state_machine_->get_debug());
    }
    kalman_filter_->update_measurement_matrix(fusion_vec_.cwiseProduct(state_machine_->get_valid_fusion_vec()));

    // buffer the measurement vector to ensure that no asynchronous measurements
    // are received during the prediction step
    const Eigen::Vector<double, TConfig::MEASUREMENT_VECTOR_SIZE> z_sync = z_;

    // set the fusion vector to zero to only consider fresh measurements in the next step
    fusion_vec_.setZero();

    // perform the prediction step of the Kalman filter
    kalman_filter_->predict(u_);

    // perform the update step of the Kalman filter
    kalman_filter_->update(z_sync);

    // get the Kalman state vector
    x_out_ = kalman_filter_->get_state_vector();

    // overwrite the state machine if the state estimation is in a safe state
    const auto& covariance = kalman_filter_->get_covariance_matrix();
    const double pos_x_variance = covariance(TConfig::STATE_POS_X_M, TConfig::STATE_POS_X_M);
    const double pos_y_variance = covariance(TConfig::STATE_POS_Y_M, TConfig::STATE_POS_Y_M);
    const double mean_position_variance = (pos_x_variance + pos_y_variance) / 2;
    if (mean_position_variance < state_estimation_params_.overwrite_pose_covariance_threshold) {
      state_machine_->update(true);
    } else {
      state_machine_->update();
    }

    // set the imu offsets
    StateEstimation<TConfig, TModel>::update_imu_offsets();
  }

  /**
   * @brief Sets the initial state of the state estimation
   *
   * @param[out] result           - bool:
   *                                True if the initial state was sucessfully set
   */
  bool set_initial_state(void) override
  {
    if (param_manager_->get_state_hash() != previous_param_state_hash_) {
      this->declare_and_update_parameters();
    }

    // set the sensor biases for the imu handler (robust to a bias_init that is shorter/longer than the input vector)
    Eigen::Vector<double, TConfig::INPUT_VECTOR_SIZE> imu_bias =
      Eigen::Vector<double, TConfig::INPUT_VECTOR_SIZE>::Zero();
    const int num_bias =
      std::min<int>(static_cast<int>(state_estimation_params_.u_bias_init.size()), TConfig::INPUT_VECTOR_SIZE);
    for (int i = 0; i < num_bias; ++i) imu_bias[i] = state_estimation_params_.u_bias_init[i];
    imu_handler_->set_sensor_bias(imu_bias);

    // state vector to overwrite the original one
    Eigen::Vector<double, TConfig::STATE_VECTOR_SIZE> x_overwrite = Eigen::VectorXd::Zero(TConfig::STATE_VECTOR_SIZE);
    bool input_valid = false;
    state_machine_->update(true);

    // Lambda to set the inital state of the kalman filter given a initial x vector
    auto set_initial_kalman_state =
      [this](const Eigen::Ref<const Eigen::Vector<double, TConfig::STATE_VECTOR_SIZE>> x_overwrite) {
        // set the initial state of the kalman filter
        this->kalman_filter_->set_state_vector(x_overwrite);
        this->kalman_filter_->set_covariance_matricies();
        this->kalman_filter_->set_initial_valid_map(state_machine_->get_debug());
        this->x_out_ = x_overwrite;

        // set the initial state of the State Machine
        // overwrite_state_machine = true allows all state transitions
        this->state_machine_->update(true);
      };

    // ensure that enough imus are valid to not trigger a safe stop after init
    if (state_machine_->get_min_valid_imus()) return false;

    // initialize the kalman filter with zeros
    if (!state_estimation_params_.initialize_from_measurements) {
      set_initial_kalman_state(x_overwrite);
      return true;
    }

    // to ensure that the initial position is valid the state vector of the kalman filter
    // is only overwritten iff the measurement is valid and updated
    fusion_vec_ = fusion_vec_.cwiseProduct(state_machine_->get_valid_fusion_vec());
    for (int i = 0; i < TConfig::NUM_POS_MEASUREMENT; ++i) {
      // clang-format off
      const Eigen::Vector<double, TConfig::POS_MEASUREMENT_VECTOR_SIZE> position_elements = fusion_vec_.segment(i * TConfig::POS_MEASUREMENT_VECTOR_SIZE, TConfig::POS_MEASUREMENT_VECTOR_SIZE); // NOLINT
      if (position_elements.isApprox(Eigen::VectorXd::Ones(TConfig::POS_MEASUREMENT_VECTOR_SIZE))) {
        x_overwrite.segment(0, TConfig::POS_MEASUREMENT_VECTOR_SIZE) = z_.segment(i * TConfig::POS_MEASUREMENT_VECTOR_SIZE, TConfig::POS_MEASUREMENT_VECTOR_SIZE); // NOLINT
        input_valid = true;
        break;
      }
      // clang-format on
    }

    const Eigen::Vector<double, 2> orientation(external_orientation_.x, external_orientation_.y);
    // clang-format off
    const int orientation_vector_offset = TConfig::MEASUREMENT_VECTOR_OFFSET_ORIENTATION + (TConfig::NUM_ORIENTATION_MEASUREMENT - 2) * TConfig::ORIENTATION_MEASUREMENT_VECTOR_SIZE; // NOLINT
    // clang-format on
    // set the orientation in the measurement vector (penultimate measurement)
    z_.segment(orientation_vector_offset, 2) = orientation;

    // set bits in the fusion vector to indicate that a new measurement was received
    fusion_vec_.segment(orientation_vector_offset, 2) = Eigen::VectorXd::Ones(2);

    // set the initial orientation of the state vector iff a valid position was found
    // we have to search for every axis individually because not every
    // orientation measurement contains all axis
    if (input_valid) {
      for (int i = 0; i < TConfig::ORIENTATION_MEASUREMENT_VECTOR_SIZE; ++i) {
        input_valid = false;
        for (int j = 0; j < TConfig::NUM_ORIENTATION_MEASUREMENT; j++) {
          // clang-format off
          const int idx = TConfig::MEASUREMENT_VECTOR_OFFSET_ORIENTATION + j * TConfig::ORIENTATION_MEASUREMENT_VECTOR_SIZE + i;  // NOLINT
          // clang-format on
          if (fusion_vec_[idx] > 0.0) {
            x_overwrite[TConfig::POS_MEASUREMENT_VECTOR_SIZE + i] = z_[idx];
            input_valid = true;
            break;
          }
        }
        if (!input_valid) break;
      }
    }

    // if a valid position and orientation was found also search for a valid velocity measurement
    if (input_valid) {
      for (int i = 0; i < TConfig::NUM_VEL_MEASUREMENT; ++i) {
        const int idx = TConfig::MEASUREMENT_VECTOR_OFFSET_VEL + i * TConfig::VEL_MEASUREMENT_VECTOR_SIZE;
        if (fusion_vec_[idx] > 0.0) {
          // clang-format off
          x_overwrite.segment(TConfig::POS_MEASUREMENT_VECTOR_SIZE + TConfig::ORIENTATION_MEASUREMENT_VECTOR_SIZE, TConfig::VEL_MEASUREMENT_VECTOR_SIZE) = z_.segment(idx, TConfig::VEL_MEASUREMENT_VECTOR_SIZE);  // NOLINT
          // clang-format on
          // set the initial state of the kalman filter
          set_initial_kalman_state(x_overwrite);
          return true;
        }
      }
      return false;
    }
    return false;
  }

  /**
   * @brief Sets the mapping from input type and num to sensor name for logging
   *
   * @param[in] mapping           - std::unordered_map:
   *                                mapping from input type and num to sensor name for logging
   */
  void set_input_config(
    const std::vector<std::pair<tam::types::state::measurements::identifier, std::string>>& mapping) override
  {
    identifier_ = mapping;
  }

  // position input
  /**
   * @brief sets the positional input
   *
   * @param[in] odometry          - tam::types::control::Odometry:
   *                                containing the odometry measurements of the position input
   * @param[in] pos_num           - uint8_t:
   *                                containing the number of the position input [0-N]
   */
  void set_input_position(const tam::types::control::Odometry& odometry, uint8_t pos_num) override
  {
    // return if the input contains a NaN
    if (odometry.position_m.hasNaN()) {
      state_machine_->set_position_invalid(pos_num);
      return;
    }

    if constexpr (!TConfig::THREE_DIMENSIONAL) {
      // sets the positional input of 2D filters
      // set the measurment vector
      if (pos_num < TConfig::NUM_POS_MEASUREMENT) {
        const int pos_measurement_idx = pos_num * TConfig::POS_MEASUREMENT_VECTOR_SIZE;
        // set position measurement vector
        z_[pos_measurement_idx + TConfig::MEASUREMENT_POS_X_M] = odometry.position_m.x;
        z_[pos_measurement_idx + TConfig::MEASUREMENT_POS_Y_M] = odometry.position_m.y;

        // set the bits corresponding to this position input in the fusion_vec to one to fuse them
        // iff the position is also valid in the state machine
        // clang-format off
        fusion_vec_.segment(pos_measurement_idx, TConfig::POS_MEASUREMENT_VECTOR_SIZE) = Eigen::VectorXd::Ones(TConfig::POS_MEASUREMENT_VECTOR_SIZE);  // NOLINT
        // clang-format on

        // update the variance of the sensor to adapt the R matrix of the kalman filter
        if (kalman_filter_->get_enable_adaptive_measurement_covariance()) {
          Eigen::Vector<double, TConfig::POS_MEASUREMENT_VECTOR_SIZE> covariance;
          covariance << odometry.pose_covariance[0], odometry.pose_covariance[7];
          kalman_filter_->set_position_covariance(covariance, pos_num);
        }
      } else {
        throw std::invalid_argument(
          "[StateEstimationCPP]: Localization input " + std::to_string(pos_num) + " does not exist");
      }
    } else {
      // sets the positional input 3D filters
      if (pos_num < TConfig::NUM_POS_MEASUREMENT) {
        const int pos_measurement_idx = pos_num * TConfig::POS_MEASUREMENT_VECTOR_SIZE;
        // set position measurement vector
        z_[pos_measurement_idx + TConfig::MEASUREMENT_POS_X_M] = odometry.position_m.x;
        z_[pos_measurement_idx + TConfig::MEASUREMENT_POS_Y_M] = odometry.position_m.y;
        z_[pos_measurement_idx + TConfig::MEASUREMENT_POS_Z_M] = odometry.position_m.z;

        // set the bits corresponding to this position input in the fusion_vec to one to fuse them
        // iff the position is also valid in the state machine
        // clang-format off
        fusion_vec_.segment(pos_measurement_idx, TConfig::POS_MEASUREMENT_VECTOR_SIZE) = Eigen::VectorXd::Ones(TConfig::POS_MEASUREMENT_VECTOR_SIZE);  // NOLINT
        // clang-format on

        // update the variance of the sensor to adapt the R matrix of the kalman filter
        if (kalman_filter_->get_enable_adaptive_measurement_covariance()) {
          Eigen::Vector<double, TConfig::POS_MEASUREMENT_VECTOR_SIZE> covariance;
          covariance << odometry.pose_covariance[0], odometry.pose_covariance[7], odometry.pose_covariance[14];
          kalman_filter_->set_position_covariance(covariance, pos_num);
        }
      } else {
        throw std::invalid_argument(
          "[StateEstimationCPP]: Position input " + std::to_string(pos_num) + " does not exist");
      }
    }
  }

  /**
   * @brief sets the status of the position input
   *
   * @param[in] status            - containing information on the status of the position input
   *                                (0: OK, 1: Warn, 2: Error, 3: STALE)
   * @param[in] pos_num           - uint8_t:
   *                                containing the number of the position input [0-N]
   */
  void set_input_position_status(const tam::types::ErrorLvl& status, uint8_t pos_num) override
  {
    // forward the status of the position input to the state machine
    if (pos_num < TConfig::NUM_POS_MEASUREMENT) {
      state_machine_->set_position_status(status, pos_num);
    } else {
      throw std::invalid_argument(
        "[StateEstimationCPP]: Position input " + std::to_string(pos_num) + " does not exist");
    }
  }

  /**
   * @brief sets the status of the position input
   *
   * @param[in] status            - containing information on the status of the position input
   *                                (0: OK, 1: Warn, 2: Error, 3: STALE)
   * @param[in] pos_num           - uint8_t:
   *                                containing the number of the position input [0-N]
   */
  void set_input_position_status(uint8_t status, uint8_t pos_num) override
  {
    // convert the unit8_t status to a tam::types::ErrorLvl and handle the input
    set_input_position_status(
      tam::type_conversions::error_type_from_diagnostic_level(static_cast<unsigned char>(status)), pos_num);
  }

  /**
   * @brief forwards a detected timeout in the position input to the state estimation state machine
   *
   * @param[in] pos_num           - uint8_t:
   *                                containing the number of the position input [0-N]
   */
  void set_position_timeout(uint8_t pos_num) override
  {
    // set the position input invalid in the state machine
    if (pos_num < TConfig::NUM_POS_MEASUREMENT) {
      state_machine_->set_position_invalid(pos_num);
    } else {
      throw std::invalid_argument(
        "[StateEstimationCPP]: Position input " + std::to_string(pos_num) + " does not exist");
    }
  }

  // orientation input
  /**
   * @brief sets the orientational input
   *
   * @param[in] odometry          - tam::types::control::Odometry:
   *                                containing the odometry measurements of the orientation input
   * @param[in] orientation_num   - uint8_t:
   *                                containing the number of the orientation input [0-N]
   * @param[in] set_roll          - bool:
   *                                sets the roll input to be fused by the kalman filter
   * @param[in] set_pitch         - bool:
   *                                sets the pitch input to be fused by the kalman filter
   * @param[in] set_yaw           - bool:
   *                                sets the yaw input to be fused by the kalman filter
   */
  void set_input_orientation(const tam::types::control::Odometry& odometry, uint8_t orientation_num, bool set_roll,
    bool set_pitch, bool set_yaw) override
  {
    // return if the input contains a NaN
    if (odometry.orientation_rad.hasNaN()) {
      state_machine_->set_orientation_invalid(orientation_num);
      return;
    }

    if constexpr (!TConfig::THREE_DIMENSIONAL) {
      // set the orientation input 2D filters
      // set the measurment vector
      if (orientation_num < TConfig::NUM_ORIENTATION_MEASUREMENT) {
        // set the yaw angle and the corresponding bit in the fusion vector to indicate
        // that the measurement was updated
        if (set_yaw) {
          // clang-format off
          const int orientation_measurement_idx = TConfig::MEASUREMENT_VECTOR_OFFSET_ORIENTATION + orientation_num * TConfig::ORIENTATION_MEASUREMENT_VECTOR_SIZE;  // NOLINT
          z_[orientation_measurement_idx + TConfig::MEASUREMENT_PSI_RAD] = odometry.orientation_rad.z;
          fusion_vec_[orientation_measurement_idx + TConfig::MEASUREMENT_PSI_RAD] = 1.0;
          // clang-format on
        }

        // update the variance of the sensor to adapt the R matrix of the kalman filter
        if (kalman_filter_->get_enable_adaptive_measurement_covariance()) {
          Eigen::Vector<double, TConfig::ORIENTATION_MEASUREMENT_VECTOR_SIZE> covariance;
          covariance << odometry.pose_covariance[35];
          kalman_filter_->set_orientation_covariance(covariance, orientation_num);
        }
      } else {
        throw std::invalid_argument(
          "[StateEstimationCPP]: Orientation input " + std::to_string(orientation_num) + " does not exist");
      }
    } else {
      // set the orientation input of 3D filters
      // set the measurment vector
      if (orientation_num < TConfig::NUM_ORIENTATION_MEASUREMENT) {
        // clang-format off
        const int orientation_measurement_idx = TConfig::MEASUREMENT_VECTOR_OFFSET_ORIENTATION + orientation_num * TConfig::ORIENTATION_MEASUREMENT_VECTOR_SIZE; // NOLINT
        // clang-format on
        // set the roll angle and the corresponding bit in the fusion vector to indicate
        // that the measurement was updated
        if (set_roll) {
          z_[orientation_measurement_idx + TConfig::MEASUREMENT_PHI_RAD] = odometry.orientation_rad.x;
          fusion_vec_[orientation_measurement_idx + TConfig::MEASUREMENT_PHI_RAD] = 1.0;
        }

        // set the pitch angle and the corresponding bit in the fusion vector to indicate
        // that the measurement was updated
        if (set_pitch) {
          z_[orientation_measurement_idx + TConfig::MEASUREMENT_THETA_RAD] = odometry.orientation_rad.y;
          fusion_vec_[orientation_measurement_idx + TConfig::MEASUREMENT_THETA_RAD] = 1.0;
        }

        // set the yaw angle and the corresponding bit in the fusion vector to indicate
        // that the measurement was updated
        if (set_yaw) {
          z_[orientation_measurement_idx + TConfig::MEASUREMENT_PSI_RAD] = odometry.orientation_rad.z;
          fusion_vec_[orientation_measurement_idx + TConfig::MEASUREMENT_PSI_RAD] = 1.0;
        }

        // update the variance of the sensor to adapt the R matrix of the kalman filter
        if (kalman_filter_->get_enable_adaptive_measurement_covariance()) {
          Eigen::Vector<double, TConfig::ORIENTATION_MEASUREMENT_VECTOR_SIZE> covariance;
          covariance << odometry.pose_covariance[21], odometry.pose_covariance[28], odometry.pose_covariance[35];
          kalman_filter_->set_orientation_covariance(covariance, orientation_num);
        }
      } else {
        throw std::invalid_argument(
          "[StateEstimationCPP]: Orientation input " + std::to_string(orientation_num) + " does not exist");
      }
    }
  }

  /**
   * @brief sets the status of the orientation input
   *
   * @param[in] status            - containing information on the status of the orientation input
   *                                (0: OK, 1: Warn, 2: Error, 3: STALE)
   * @param[in] orientation_num   - uint8_t:
   *                                containing the number of the orientation input [0-N]
   */
  void set_input_orientation_status(const tam::types::ErrorLvl& status, uint8_t orientation_num) override
  {
    // forward the status of the orientation input to the state machine
    if (orientation_num < TConfig::NUM_ORIENTATION_MEASUREMENT) {
      state_machine_->set_orientation_status(status, orientation_num);
    } else {
      throw std::invalid_argument(
        "[StateEstimationCPP]: Orientation input " + std::to_string(orientation_num) + " does not exist");
    }
  }

  /**
   * @brief sets the status of the orientation input
   *
   * @param[in] status            - containing information on the status of the orientation input
   *                                (0: OK, 1: Warn, 2: Error, 3: STALE)
   * @param[in] orientation_num   - uint8_t:
   *                                containing the number of the orientation input [0-N]
   */
  void set_input_orientation_status(uint8_t status, uint8_t orientation_num) override
  {
    // convert the unit8_t status to a tam::types::ErrorLvl and handle the input
    set_input_orientation_status(
      tam::type_conversions::error_type_from_diagnostic_level(static_cast<unsigned char>(status)), orientation_num);
  }

  /**
   * @brief forwards a detected timeout in the orientation input to the state estimation state machine
   *
   * @param[in] orientation_num   - uint8_t:
   *                                containing the number of the orientation input [0-N]
   */
  void set_orientation_timeout(uint8_t orientation_num) override
  {
    // set the orientation input invalid in the state machine
    if (orientation_num < TConfig::NUM_ORIENTATION_MEASUREMENT) {
      state_machine_->set_orientation_invalid(orientation_num);
    } else {
      throw std::invalid_argument(
        "[StateEstimationCPP]: Orientation input " + std::to_string(orientation_num) + " does not exist");
    }
  }

  // linear velocity inputs
  /**
   * @brief sets the linear velocity input
   *
   * @param[in] odometry          - tam::types::control::Odometry:
   *                                containing the linear velocity measured
   * @param[in] vel_num           - uint8_t:
   *                                containing the number of the velocity input [0-N]
   */
  void set_input_linear_velocity(const tam::types::control::Odometry& odometry, uint8_t vel_num) override
  {
    // return if the input contains a NaN
    if (odometry.velocity_mps.hasNaN()) {
      state_machine_->set_linear_velocity_invalid(vel_num);
      return;
    }

    if constexpr (!TConfig::THREE_DIMENSIONAL) {
      // sets the linear velocity input in 2D
      // set the measurment vector
      if (vel_num < TConfig::NUM_VEL_MEASUREMENT) {
        // clang-format off
        const int vel_measurement_idx = TConfig::MEASUREMENT_VECTOR_OFFSET_VEL + vel_num * TConfig::VEL_MEASUREMENT_VECTOR_SIZE;  // NOLINT
        // transform the 3D linear velocities into the footprint frame
        tam::types::common::Vector3D<double> odometry_footprint;
        odometry_footprint = tam::helpers::euler_rotations::vector_to_2d(odometry.velocity_mps, external_orientation_.y, external_orientation_.x); // NOLINT
        // set the linear velocity measurements
        z_[vel_measurement_idx + TConfig::MEASUREMENT_VX_MPS] = odometry_footprint.x;
        z_[vel_measurement_idx + TConfig::MEASUREMENT_VY_MPS] = odometry_footprint.y;
        // set the bits corresponding to this linear velocity input in the fusion_vec to one
        // to fuse them iff the localization is also valid in the state machine
        fusion_vec_.segment(vel_measurement_idx, TConfig::VEL_MEASUREMENT_VECTOR_SIZE) = Eigen::VectorXd::Ones(TConfig::VEL_MEASUREMENT_VECTOR_SIZE);  // NOLINT
        // clang-format on
      } else {
        throw std::invalid_argument(
          "[StateEstimationCPP]: Linear velocity input " + std::to_string(vel_num) + " does not exist");
      }
    } else {
      // set the linear velocity input in 3D
      // set the measurment vector
      if (vel_num < TConfig::NUM_VEL_MEASUREMENT) {
        // clang-format off
        const int vel_measurement_idx = TConfig::MEASUREMENT_VECTOR_OFFSET_VEL + vel_num * TConfig::VEL_MEASUREMENT_VECTOR_SIZE;  // NOLINT
        const double outlier_threshold = state_estimation_params_.hard_linear_velocity_outlier_th;
        // set the linear velocity measurements
        z_[vel_measurement_idx + TConfig::MEASUREMENT_VX_MPS] = odometry.velocity_mps.x;
        z_[vel_measurement_idx + TConfig::MEASUREMENT_VY_MPS] = odometry.velocity_mps.y;
        z_[vel_measurement_idx + TConfig::MEASUREMENT_VZ_MPS] = odometry.velocity_mps.z;
        // completly reject measurements that violate the hard treshold
        const Eigen::Vector<double, TConfig::VEL_MEASUREMENT_VECTOR_SIZE> outlier_distance_se = tam::core::state::outlier_rejection::squared_distance(x_out_.segment(TConfig::STATE_VX_MPS, TConfig::VEL_MEASUREMENT_VECTOR_SIZE), odometry.velocity_mps);  // NOLINT

        if ((outlier_distance_se.array() < outlier_threshold).all() || (0.0 == x_out_.segment(TConfig::STATE_VX_MPS, TConfig::VEL_MEASUREMENT_VECTOR_SIZE).array()).all()) {  // NOLINT
          // set the bits corresponding to this linear velocity input in the fusion_vec to one
          // to fuse them iff the localization is also valid in the state machine
          fusion_vec_.segment(vel_measurement_idx, TConfig::VEL_MEASUREMENT_VECTOR_SIZE) = Eigen::VectorXd::Ones(TConfig::VEL_MEASUREMENT_VECTOR_SIZE);  // NOLINT
          outlier_buffer_[vel_num] = 0;
        } else {
          fusion_vec_.segment(vel_measurement_idx, TConfig::VEL_MEASUREMENT_VECTOR_SIZE) = Eigen::VectorXd::Zero(TConfig::VEL_MEASUREMENT_VECTOR_SIZE);  // NOLINT
          outlier_buffer_[vel_num]++;
        }
        // clang-format on
      } else {
        throw std::invalid_argument(
          "[StateEstimationCPP]: Linear velocity input " + std::to_string(vel_num) + " does not exist");
      }
    }
  }

  /**
   * @brief sets the status of the linear velocity input
   *
   * @param[in] status            - containing information on the status of the linear velocity input
   *                                (0: OK, 1: Warn, 2: Error, 3: STALE)
   * @param[in] vel_num           - uint8_t:
   *                                containing the number of the velocity input [0-N]
   */
  void set_input_linear_velocity_status(const tam::types::ErrorLvl& status, uint8_t vel_num) override
  {
    // forward the status of the linear velocity input to the state machine
    if (vel_num < TConfig::NUM_VEL_MEASUREMENT) {
      const int max_outlier_threshold = state_estimation_params_.max_consecutive_vel_hard_outliers;
      if (outlier_buffer_[vel_num] < max_outlier_threshold) {
        state_machine_->set_linear_velocity_status(status, vel_num);
      } else {
        state_machine_->set_linear_velocity_status(tam::types::ErrorLvl::ERROR, vel_num);
      }
    } else {
      throw std::invalid_argument(
        "[StateEstimationCPP]: Linear velocity input " + std::to_string(vel_num) + " does not exist");
    }
  }

  /**
   * @brief sets the status of the linear velocity input
   *
   * @param[in] status            - containing information on the status of the linear velocity input
   *                                (0: OK, 1: Warn, 2: Error, 3: STALE)
   * @param[in] vel_num           - uint8_t:
   *                                containing the number of the velocity input [0-N]
   */
  void set_input_linear_velocity_status(uint8_t status, uint8_t vel_num) override
  {
    // convert the unit8_t status to a tam::types::ErrorLvl and handle the input
    set_input_linear_velocity_status(
      tam::type_conversions::error_type_from_diagnostic_level(static_cast<unsigned char>(status)), vel_num);
  }

  /**
   * @brief forwards a detected timeout in the linear velocity input to the state estimation state machine
   *
   * @param[in] vel_num           - uint8_t:
   *                                containing the number of the velocity input [0-N]
   */
  void set_linear_velocity_timeout(uint8_t vel_num) override
  {
    // set the linear velocity input invalid in the state machine
    if (vel_num < TConfig::NUM_VEL_MEASUREMENT) {
      state_machine_->set_linear_velocity_invalid(vel_num);
    } else {
      throw std::invalid_argument(
        "[StateEstimationCPP]: Linear velocity input " + std::to_string(vel_num) + " does not exist");
    }
  }

  // IMU inputs
  /**
   * @brief sets the linear acceleration input
   *
   * @param[in] acceleration      - tam::types::control::AccelerationwithCovariances:
   *                                containing the acceleration of a IMU
   * @param[in] imu_num           - uint8_t:
   *                                containing the number of the imu input [0-N]
   */
  void set_input_acceleration(
    const tam::types::control::AccelerationwithCovariances& acceleration, uint8_t imu_num) override
  {
    // return if the input contains a NaN
    if (acceleration.acceleration_mps2.hasNaN()) {
      state_machine_->set_imu_invalid(imu_num);
      return;
    }

    if constexpr (!TConfig::THREE_DIMENSIONAL) {
      // set the raw input vector
      // set the linear acceleration input in 2D
      if (imu_num < (TConfig::NUM_IMU_MEASUREMENT + TConfig::NUM_BACKUP_IMU_MEASUREMENT)) {
        const int imu_input_idx = imu_num * TConfig::INPUT_VECTOR_SIZE;
        // set the linear acceleration of the raw input vector
        u_raw_[imu_input_idx + TConfig::INPUT_AX_MPS2] = acceleration.acceleration_mps2.x;
        u_raw_[imu_input_idx + TConfig::INPUT_AY_MPS2] = acceleration.acceleration_mps2.y;
        u_cov_[imu_input_idx + TConfig::INPUT_AX_MPS2] = acceleration.acceleration_covariance[0];
        u_cov_[imu_input_idx + TConfig::INPUT_AY_MPS2] = acceleration.acceleration_covariance[7];
        // preprocess the input vector containing linear accelerations and angular veocities
        // the valid_u_fusion_vec ensures that only valid measurements are fused
        if (updated_angular_velocity_[imu_num] == 1) {
          // clang-format off
          u_raw_.segment(imu_input_idx, TConfig::INPUT_VECTOR_SIZE) = imu_handler_->filter_imu_measurements(u_raw_.segment(imu_input_idx, TConfig::INPUT_VECTOR_SIZE), imu_num);  // NOLINT
          // clang-format on
          updated_angular_velocity_[imu_num] = 0;
        } else {
          updated_acceleration_[imu_num] = 1;
        }
      } else {
        throw std::invalid_argument("[StateEstimationCPP]: IMU input " + std::to_string(imu_num) + " does not exist");
      }
    } else {
      // set the linear acceleration input in 3D
      if (imu_num < (TConfig::NUM_IMU_MEASUREMENT + TConfig::NUM_BACKUP_IMU_MEASUREMENT)) {
        // clang-format off
        const int imu_input_idx = imu_num * TConfig::INPUT_VECTOR_SIZE;
        const double outlier_threshold = state_estimation_params_.hard_accelerometer_outlier_th;
        // completly reject measurements that violate the hard treshold
        const Eigen::Vector<double, 3> outlier_distance = tam::core::state::outlier_rejection::squared_distance(u_.segment(TConfig::INPUT_AX_MPS2, 3), acceleration.acceleration_mps2);  // NOLINT
        if ((outlier_threshold > outlier_distance.array()).all() || (0.0 == u_.segment(TConfig::INPUT_AX_MPS2, 3).array()).all() || imu_num >= TConfig::NUM_IMU_MEASUREMENT) {  // NOLINT
          // set the linear acceleration of the raw input vector
          u_raw_[imu_input_idx + TConfig::INPUT_AX_MPS2] = acceleration.acceleration_mps2.x;
          u_raw_[imu_input_idx + TConfig::INPUT_AY_MPS2] = acceleration.acceleration_mps2.y;
          u_raw_[imu_input_idx + TConfig::INPUT_AZ_MPS2] = acceleration.acceleration_mps2.z;
          u_cov_[imu_input_idx + TConfig::INPUT_AX_MPS2] = acceleration.acceleration_covariance[0];
          u_cov_[imu_input_idx + TConfig::INPUT_AY_MPS2] = acceleration.acceleration_covariance[7];
          u_cov_[imu_input_idx + TConfig::INPUT_AZ_MPS2] = acceleration.acceleration_covariance[14];
          outlier_buffer_[TConfig::NUM_VEL_MEASUREMENT + TConfig::NUM_IMU_MEASUREMENT + imu_num] = 0;
        } else {
          outlier_buffer_[TConfig::NUM_VEL_MEASUREMENT + TConfig::NUM_IMU_MEASUREMENT + imu_num]++;
        }
        // preprocess the input vector containing linear accelerations and angular veocities
        // the valid_u_fusion_vec ensures that only valid measurements are fused
        if (updated_angular_velocity_[imu_num] == 1) {
          u_raw_.segment(imu_input_idx, TConfig::INPUT_VECTOR_SIZE) = imu_handler_->filter_imu_measurements(u_raw_.segment(imu_input_idx, TConfig::INPUT_VECTOR_SIZE), imu_num);  // NOLINT
          updated_angular_velocity_[imu_num] = 0;
        } else {
          updated_acceleration_[imu_num] = 1;
        }
        // clang-format on
      } else {
        throw std::invalid_argument("[StateEstimationCPP]: IMU input " + std::to_string(imu_num) + " does not exist");
      }
    }
  }

  /**
   * @brief sets the angular velocity input
   *
   * @param[in] odometry          - tam::types::control::Odometry:
   *                                containing the angular velocity of a IMU
   * @param[in] imu_num           - uint8_t:
   *                                containing the number of the imu input [0-N]
   */
  void set_input_angular_velocity(const tam::types::control::Odometry& odometry, uint8_t imu_num) override
  {
    // return if the input contains a NaN
    if (odometry.angular_velocity_radps.hasNaN()) {
      state_machine_->set_imu_invalid(imu_num);
      return;
    }

    if constexpr (!TConfig::THREE_DIMENSIONAL) {
      // set the angular velocity input in 2D
      // set the raw input vector
      if (imu_num < (TConfig::NUM_IMU_MEASUREMENT + TConfig::NUM_BACKUP_IMU_MEASUREMENT)) {
        const int imu_input_idx = imu_num * TConfig::INPUT_VECTOR_SIZE;
        // set yaw rate measurement of the raw input vector
        u_raw_[imu_input_idx + TConfig::INPUT_DPSI_RADPS] = odometry.angular_velocity_radps.z;
        u_cov_[imu_input_idx + TConfig::INPUT_DPSI_RADPS] = odometry.velocity_covariance[35];
        // preprocess the input vector containing linear accelerations and angular veocities
        // the valid_u_fusion_vec ensures that only valid measurements are fused
        if (updated_acceleration_[imu_num] == 1) {
          // clang-format off
          u_raw_.segment(imu_input_idx, TConfig::INPUT_VECTOR_SIZE) = imu_handler_->filter_imu_measurements(u_raw_.segment(imu_input_idx, TConfig::INPUT_VECTOR_SIZE), imu_num);  // NOLINT
          // clang-format on
          updated_acceleration_[imu_num] = 0;
        } else {
          updated_angular_velocity_[imu_num] = 1;
        }
      } else {
        throw std::invalid_argument("[StateEstimationCPP]: IMU input " + std::to_string(imu_num) + " does not exist");
      }
    } else {
      // set the angular velocity input in 3D
      // set the raw input vector
      if (imu_num < (TConfig::NUM_IMU_MEASUREMENT + TConfig::NUM_BACKUP_IMU_MEASUREMENT)) {
        // clang-format off
        const int imu_input_idx = imu_num * TConfig::INPUT_VECTOR_SIZE;
        const double outlier_threshold = state_estimation_params_.hard_angular_velocity_outlier_th;
        // completly reject measurements that violate the hard treshold
        const Eigen::Vector<double, 3> outlier_distance = tam::core::state::outlier_rejection::squared_distance(u_.segment(TConfig::INPUT_DPHI_RADPS, 3), odometry.angular_velocity_radps);  // NOLINT
        if ((outlier_threshold > outlier_distance.array()).all() || (0.0 == u_.segment(TConfig::INPUT_DPHI_RADPS, 3).array()).all() || imu_num >= TConfig::NUM_IMU_MEASUREMENT) {  // NOLINT
          // set the angular velocity of the raw input vector
          u_raw_[imu_input_idx + TConfig::INPUT_DPHI_RADPS] = odometry.angular_velocity_radps.x;
          u_raw_[imu_input_idx + TConfig::INPUT_DTHETA_RADPS] = odometry.angular_velocity_radps.y;
          u_raw_[imu_input_idx + TConfig::INPUT_DPSI_RADPS] = odometry.angular_velocity_radps.z;
          u_cov_[imu_input_idx + TConfig::INPUT_DPHI_RADPS] = odometry.velocity_covariance[21];
          u_cov_[imu_input_idx + TConfig::INPUT_DTHETA_RADPS] = odometry.velocity_covariance[28];
          u_cov_[imu_input_idx + TConfig::INPUT_DPSI_RADPS] = odometry.velocity_covariance[35];
          outlier_buffer_[TConfig::NUM_VEL_MEASUREMENT + imu_num] = 0;
        } else {
          outlier_buffer_[TConfig::NUM_VEL_MEASUREMENT + imu_num]++;
        }
        // preprocess the input vector containing linear accelerations and angular veocities
        // the valid_u_fusion_vec ensures that only valid measurements are fused
        if (updated_acceleration_[imu_num] == 1) {
          u_raw_.segment(imu_input_idx, TConfig::INPUT_VECTOR_SIZE) = imu_handler_->filter_imu_measurements(u_raw_.segment(imu_input_idx, TConfig::INPUT_VECTOR_SIZE), imu_num);  // NOLINT
          updated_acceleration_[imu_num] = 0;
        } else {
          updated_angular_velocity_[imu_num] = 1;
        }
        // clang-format on
      } else {
        throw std::invalid_argument("[StateEstimationCPP]: IMU input " + std::to_string(imu_num) + " does not exist");
      }
    }
  }

  /**
   * @brief sets the status of the imu input
   *
   * @param[in] status            - containing information on the status of the imu input
   *                                (0: OK, 1: Warn, 2: Error, 3: STALE)
   * @param[in] imu_num           - uint8_t:
   *                                containing the number of the imu input [0-N]
   */
  void set_input_imu_status(const tam::types::ErrorLvl& status, uint8_t imu_num) override
  {
    const int max_outlier_threshold = state_estimation_params_.max_consecutive_imu_hard_outliers;
    tam::types::ErrorLvl imu_status = status;

    // outlier rejection
    if (imu_num < TConfig::NUM_IMU_MEASUREMENT) {
      const int angular_velocity_outlier_idx = TConfig::NUM_VEL_MEASUREMENT + imu_num;
      const int acceleration_outlier_idx = TConfig::NUM_VEL_MEASUREMENT + TConfig::NUM_IMU_MEASUREMENT + imu_num;
      // clang-format off
      if (outlier_buffer_[angular_velocity_outlier_idx] > max_outlier_threshold || outlier_buffer_[acceleration_outlier_idx] > max_outlier_threshold) {  // NOLINT
        imu_status = tam::types::ErrorLvl::ERROR;
      }
      // clang-format on
    }

    if (imu_num < (TConfig::NUM_IMU_MEASUREMENT + TConfig::NUM_BACKUP_IMU_MEASUREMENT)) {
      state_machine_->set_imu_status(imu_status, imu_num);
    } else {
      throw std::invalid_argument("[StateEstimationCPP]: IMU input " + std::to_string(imu_num) + " does not exist");
    }
  }

  /**
   * @brief sets the status of the imu input
   *
   * @param[in] status            - containing information on the status of the imu input
   *                                (0: OK, 1: Warn, 2: Error, 3: STALE)
   * @param[in] imu_num           - uint8_t:
   *                                containing the number of the imu input [0-N]
   */
  void set_input_imu_status(uint8_t status, uint8_t imu_num) override
  {
    // convert the unit8_t status to a tam::types::ErrorLvl and handle the input
    set_input_imu_status(
      tam::type_conversions::error_type_from_diagnostic_level(static_cast<unsigned char>(status)), imu_num);
  }

  /**
   * @brief forwards a detected timeout in the imu input to the state estimation state machine
   *
   * @param[in] imu_num           - uint8_t:
   *                                containing the number of the imu input [0-N]
   */
  void set_imu_timeout(uint8_t imu_num) override
  {
    // set the imu input invalid in the state machine
    if (imu_num < (TConfig::NUM_IMU_MEASUREMENT + TConfig::NUM_BACKUP_IMU_MEASUREMENT)) {
      state_machine_->set_imu_invalid(imu_num);
    } else {
      throw std::invalid_argument("[StateEstimationCPP]: IMU input " + std::to_string(imu_num) + " does not exist");
    }
  }

  // Wheelspeed input
  /**
   * @brief sets the angular velocity of the wheels as input to convert them into linear velocities
   *
   * @param[in] wheel             - tam::types::common::DataPerWheel<double>:
   *                                angular velocity per wheel
   */
  void set_input_wheelspeeds(const tam::types::common::DataPerWheel<double>& wheel) override
  {
    // update the wheel speed odometry
    vehicle_model_handler_->input_wheel_angular_velocities(wheel);

    // set the linear velocity in the measurement vector (last measurement)
    set_input_linear_velocity(vehicle_model_handler_->get_vehicle_model_odometry(), TConfig::NUM_VEL_MEASUREMENT - 1);

    // buffer the vehicle model output for debuging
    vehicle_model_velocity_ = vehicle_model_handler_->get_vehicle_model_odometry().velocity_mps;
  }

  /**
   * @brief sets the status of the wheelspeed input
   *
   * @param[in] status            - containing information on the status of the wheelspeed input
   *                                (0: OK, 1: Warn, 2: Error, 3: STALE)
   */
  void set_input_wheelspeed_status(const tam::types::ErrorLvl& status) override
  {
    // update the status of the vehicle model handler
    vehicle_model_handler_->input_wheelspeed_status(status);

    // forward the updated status to the state machine
    set_input_linear_velocity_status(vehicle_model_handler_->get_status(), TConfig::NUM_VEL_MEASUREMENT - 1);
  }

  /**
   * @brief sets the status of the wheelspeed input
   *
   * @param[in] status            - containing information on the status of the wheelspeed input
   *                                (0: OK, 1: Warn, 2: Error, 3: STALE)
   */
  void set_input_wheelspeed_status(uint8_t status) override
  {
    // convert the unit8_t status to a tam::types::ErrorLvl and handle the input
    set_input_wheelspeed_status(
      tam::type_conversions::error_type_from_diagnostic_level(static_cast<unsigned char>(status)));
  }

  /**
   * @brief forwards a detected timeout in the wheelspeed input to the state estimation state machine
   */
  void set_wheelspeeds_timeout(void) override
  {
    // update the status of the vehicle model handler
    vehicle_model_handler_->input_wheelspeed_status(tam::types::ErrorLvl::ERROR);

    // set the the last velocity input invalid in the state machine
    state_machine_->set_linear_velocity_invalid(TConfig::NUM_VEL_MEASUREMENT - 1);
  }

  // Steering angle input
  /**
   * @brief sets the measured steering angle to use it in the vehicle model
   *
   * @param[in] steering_angle    - double:
   *                                steering angle in rad
   */
  void set_input_steering_angle(double steering_angle) override
  {
    // update the steering angle in the vehicle model
    vehicle_model_handler_->input_steering_angle(steering_angle);
  }

  /**
   * @brief sets the status of the steering angle input
   *
   * @param[in] status            - containing information on the status of the steering angle input
   *                                (0: OK, 1: Warn, 2: Error, 3: STALE)
   */
  void set_input_steering_angle_status(const tam::types::ErrorLvl& status) override
  {
    // update the status of the vehicle model handler
    vehicle_model_handler_->input_steering_angle_status(status);

    // forward the updated status to the state machine
    set_input_linear_velocity_status(vehicle_model_handler_->get_status(), TConfig::NUM_VEL_MEASUREMENT - 1);
  }

  /**
   * @brief sets the status of the steering angle input
   *
   * @param[in] status            - containing information on the status of the steering angle input
   *                                (0: OK, 1: Warn, 2: Error, 3: STALE)
   */
  void set_input_steering_angle_status(uint8_t status) override
  {
    // convert the unit8_t status to a tam::types::ErrorLvl and handle the input
    set_input_steering_angle_status(
      tam::type_conversions::error_type_from_diagnostic_level(static_cast<unsigned char>(status)));
  }

  /**
   * @brief forwards a detected timeout in the steering angle input to the state estimation state machine
   */
  void set_steering_angle_timeout(void) override
  {
    // update the status of the vehicle model handler
    vehicle_model_handler_->input_steering_angle_status(tam::types::ErrorLvl::ERROR);

    // set the the last velocity input invalid in the state machine
    // if the steering angle is used in the vehicle model handler
    if (vehicle_model_handler_->get_status() != tam::types::ErrorLvl::OK) {
      state_machine_->set_linear_velocity_invalid(TConfig::NUM_VEL_MEASUREMENT - 1);
    }
  }

  // External orientation prior input
  /**
   * @brief sets the external orientation prior used to compensate the IMU measurements
   *
   * @param[in] external_orientation - tam::types::common::Vector3D<double>:
   *                                   vector containing the external orientation prior
   */
  void set_input_external_orientation(
    const tam::types::common::Vector3D<double>& external_orientation) override
  {
    external_orientation_ = external_orientation;
    if constexpr (TConfig::THREE_DIMENSIONAL) {
      // additionally fuse the external orientation prior to improve the orientation prediction for
      // 3D filters
      if (state_estimation_params_.fuse_external_orientation) {
        Eigen::Vector<double, 2> orientation(external_orientation.x, external_orientation.y);
        // clang-format off
        // set the orientation in the measurement vector (penultimate measurement)
        z_.segment(TConfig::MEASUREMENT_VECTOR_OFFSET_ORIENTATION + (TConfig::NUM_ORIENTATION_MEASUREMENT - 2) * TConfig::ORIENTATION_MEASUREMENT_VECTOR_SIZE, 2) = orientation;  // NOLINT
        // set bits in the fusion vector to indicate that a new measurement was received
        fusion_vec_.segment(TConfig::MEASUREMENT_VECTOR_OFFSET_ORIENTATION + (TConfig::NUM_ORIENTATION_MEASUREMENT - 2) * TConfig::ORIENTATION_MEASUREMENT_VECTOR_SIZE, 2) = Eigen::VectorXd::Ones(2);  // NOLINT
        // clang-format on
      }
    }
  }

  /**
   * @brief sets the status for the external orientation prior
   *
   * @param[in] status            - containing information on the status of the external
   *                                orientation prior input (0: OK, 1: Warn, 2: Error, 3: STALE)
   */
  void set_input_external_orientation_status(const tam::types::ErrorLvl& status) override
  {
    // forward the status of the penultimate orientation input to the state machine
    // if the external orientation prior should be fused
    if (state_estimation_params_.fuse_external_orientation) {
      state_machine_->set_orientation_status(status, TConfig::NUM_ORIENTATION_MEASUREMENT - 2);
    }
  }

  /**
   * @brief sets the status for the external orientation prior
   *
   * @param[in] status            - containing information on the status of the external
   *                                orientation prior input (0: OK, 1: Warn, 2: Error, 3: STALE)
   */
  void set_input_external_orientation_status(uint8_t status) override
  {
    // convert the unit8_t status to a tam::types::ErrorLvl and handle the input
    set_input_external_orientation_status(
      tam::type_conversions::error_type_from_diagnostic_level(static_cast<unsigned char>(status)));
  }

  // Non state estimation specific output
  /**
   * @brief returns a pointer to the param manager composer
   *
   * @param[out]                  - tam::pmg::ParamManagerComposer::SharedPtr
   */
  tam::pmg::ParamManagerComposer::SharedPtr get_param_handler(void) override { return param_manager_composer_; }

  // output state estimation
  /**
   * @brief returns the odometry output predicted by the state estimation
   *
   * @param[out]                  - tam::types::control::Odometry
   */
  tam::types::control::Odometry get_odometry(void) override
  {
    // map state estimation output to odometry type
    tam::types::control::Odometry output;
    if constexpr (!TConfig::THREE_DIMENSIONAL) {
      // odometry output of 2D filters
      // get the covariance matrix of the kalman filter
      // clang-format off
      const Eigen::Matrix<double, TConfig::STATE_VECTOR_SIZE, TConfig::STATE_VECTOR_SIZE>& P = kalman_filter_->get_covariance_matrix();  // NOLINT
      // clang-format on

      // x,y,z coordintates
      output.position_m.x = x_out_[TConfig::STATE_POS_X_M];
      output.position_m.y = x_out_[TConfig::STATE_POS_Y_M];

      // roll, pitch, yaw
      output.orientation_rad.x = 0.0;
      output.orientation_rad.y = 0.0;
      output.orientation_rad.z = x_out_[TConfig::STATE_PSI_RAD];

      // linear velocity
      output.velocity_mps.x = x_out_[TConfig::STATE_VX_MPS];
      output.velocity_mps.y = x_out_[TConfig::STATE_VY_MPS];

      // angluar velocity
      output.angular_velocity_radps.z = u_[TConfig::INPUT_DPSI_RADPS];

      // set the covariance matrix for the position
      output.pose_covariance[0] = P(0, 0);
      output.pose_covariance[1] = P(0, 1);
      output.pose_covariance[6] = P(1, 0);
      output.pose_covariance[7] = P(1, 1);

      // set the covariance matrix for the orientation
      output.pose_covariance[35] = P(2, 2);

      // set the covariance matrix for the linear velocity
      output.velocity_covariance[0] = P(3, 3);
      output.velocity_covariance[1] = P(3, 4);
      output.velocity_covariance[6] = P(4, 3);
      output.velocity_covariance[7] = P(4, 4);
    } else {
      // odometry output of 3D filters
      // get the covariance matrix of the kalman filter
      // clang-format off
      const Eigen::Matrix<double, TConfig::STATE_VECTOR_SIZE, TConfig::STATE_VECTOR_SIZE>& P = kalman_filter_->get_covariance_matrix();  // NOLINT
      // clang-format on

      // x,y,z coordintates
      output.position_m.x = x_out_[TConfig::STATE_POS_X_M];
      output.position_m.y = x_out_[TConfig::STATE_POS_Y_M];
      output.position_m.z = x_out_[TConfig::STATE_POS_Z_M];

      // roll, pitch, yaw
      output.orientation_rad.x = x_out_[TConfig::STATE_PHI_RAD];
      output.orientation_rad.y = x_out_[TConfig::STATE_THETA_RAD];
      output.orientation_rad.z = x_out_[TConfig::STATE_PSI_RAD];

      // covariance matrix for position and orientation
      for (int i = 0; i < 6; ++i) {
        for (int j = 0; j < 6; ++j) {
          output.pose_covariance[i * 6 + j] = P(i, j);
        }
      }

      // linear velocity
      output.velocity_mps.x = x_out_[TConfig::STATE_VX_MPS];
      output.velocity_mps.y = x_out_[TConfig::STATE_VY_MPS];
      output.velocity_mps.z = x_out_[TConfig::STATE_VZ_MPS];

      // angluar velocity
      output.angular_velocity_radps.x = u_[TConfig::INPUT_DPHI_RADPS];
      output.angular_velocity_radps.y = u_[TConfig::INPUT_DTHETA_RADPS];
      output.angular_velocity_radps.z = u_[TConfig::INPUT_DPSI_RADPS];

      // linear velocity covariance
      for (int i = 6; i < 9; ++i) {
        for (int j = 6; j < 9; ++j) {
          output.velocity_covariance[(i - 6) * 6 + j - 6] = P(i, j);
        }
      }
    }
    return output;
  }

  /**
   * @brief returns the linear acceleration output predicted by the state estimation
   *
   * @param[out]                  - tam::types::control::AccelerationwithCovariances
   */
  tam::types::control::AccelerationwithCovariances get_acceleration(void) override
  {
    // map state estimation output to odometry type
    tam::types::control::AccelerationwithCovariances output;

    // linear accelerations in x and y
    output.acceleration_mps2.x = u_[TConfig::INPUT_AX_MPS2];
    output.acceleration_mps2.y = u_[TConfig::INPUT_AY_MPS2];

    if constexpr (TConfig::THREE_DIMENSIONAL) {
      // additionally output the vertical acceleration for 3D filters
      output.acceleration_mps2.z = u_[TConfig::INPUT_AZ_MPS2];
    }

    return output;
  }

  /**
   * @brief returns the state estimation status
   *
   * @param[out]                  - tam::types::ErrorLvl
   *                                (0: OK, 1: Warn, 2: Error, 3: STALE)
   */
  tam::types::ErrorLvl get_status(void) override { return state_machine_->get_state(); }

  /**
   * @brief returns the state estimation status
   *
   * @param[out]                  - uint8_t:
   *                                (0: OK, 1: Warn, 2: Error, 3: STALE)
   */
  uint8_t get_status_as_uint8_t(void) override
  {
    return static_cast<uint8_t>(tam::type_conversions::diagnostic_level_from_type(state_machine_->get_state()));
  }

  /**
   * @brief returns the sideslip angle (Schwimmwinkel) predicted by the state estimation
   *
   * @param[out]                  - double
   */
  double get_sideslip_angle(void) override
  {
    // calculate the sideslip angle based on vx and vy in the vehicle coordinate frame
    if (x_out_[TConfig::STATE_VX_MPS] > 3.0) {
      return std::atan2(x_out_[TConfig::STATE_VY_MPS], x_out_[TConfig::STATE_VX_MPS]);
    } else {
      return 0.0;
    }
  }

  /**
   * @brief returns a string containing the status message generated by the state machine
   *
   * @param[out]                  - std::string
   */
  std::string get_state_machine_status_msg(void) override
  {
    // forward the status message of the state machine
    return state_machine_->get_status_msg();
  }

  /**
   * @brief returns a debug container containing the state machine debug values
   *
   * @param[in]                   - tam::tsl::ValueLogger::SharedPtr
   */
  void get_state_machine_debug_output(tam::tsl::ValueLogger::SharedPtr logger) override
  {
    // get the state machine debug output containing the valid bits
    const auto& state_machine_debug = state_machine_->get_debug();

    // Iterate through the map
    std::string debug_namespace;
    debug_namespace.reserve(64);
    for (const auto& [id, prefix] : identifier_) {
      const auto it = state_machine_debug.find(id);
      if (it != state_machine_debug.end()) {
        debug_namespace = "state_machine/valid_";
        if (id.type == tam::types::state::measurements::POS) debug_namespace += "position/";
        if (id.type == tam::types::state::measurements::ORIENTATION) debug_namespace += "orientation/";
        if (id.type == tam::types::state::measurements::VEL) debug_namespace += "velocity/";
        if (id.type == tam::types::state::measurements::IMU) debug_namespace += "imu/";
        logger->log(debug_namespace + prefix, it->second);
      }
    }

    logger->log("state_machine/overall_state",
      static_cast<uint8_t>(tam::type_conversions::diagnostic_level_from_type(state_machine_->get_state())));
  }

  /**
   * @brief returns a debug container containing the kalman filter debug values
   *
   * @param[in]                   - tam::tsl::ValueLogger::SharedPtr
   */
  void get_kalman_filter_debug_output(tam::tsl::ValueLogger::SharedPtr logger) override
  {
    // get the kalman filter debug output
    const auto& kalman_filter_debug = kalman_filter_->get_debug();

    // Iterate through the map
    std::string debug_namespace;
    debug_namespace.reserve(64);
    for (const auto& [id, prefix] : identifier_) {
      if (id.type == tam::types::state::measurements::IMU) continue;
      const auto it = kalman_filter_debug.find(id);
      if (it != kalman_filter_debug.end()) {
        debug_namespace = "kalman_filter/" + prefix + "/";
        for (const auto& debug_pair : it->second) {
          logger->log(debug_namespace + debug_pair.first, debug_pair.second);
        }
      }
    }

    // log 2D vehicle state estimate in body frame
    logger->log("model/vx_mps", vehicle_model_velocity_.x);
    logger->log("model/vy_mps", vehicle_model_velocity_.y);
    logger->log("model/dyn_tire_radii_scale", vehicle_model_handler_->get_dyn_tire_radii_scale());

    if constexpr (TConfig::THREE_DIMENSIONAL) {
      // additional logging for 3D filters
      logger->log("external_orientation/phi", external_orientation_.x);
      logger->log("external_orientation/theta", external_orientation_.y);
      logger->log("ref_angles/phi", ref_angles_.x);
      logger->log("ref_angles/theta", ref_angles_.y);

      // log the imu bias
      const Eigen::Vector<double, TConfig::INPUT_VECTOR_SIZE> bias = imu_handler_->get_sensor_bias();
      logger->log("bias/dphi_rads", bias[TConfig::INPUT_DPHI_RADPS]);
      logger->log("bias/dtheta_rads", bias[TConfig::INPUT_DTHETA_RADPS]);
      logger->log("bias/dpsi_rads", bias[TConfig::INPUT_DPSI_RADPS]);

      logger->log("bias/ax_mps2", bias[TConfig::INPUT_AX_MPS2]);
      logger->log("bias/ay_mps2", bias[TConfig::INPUT_AY_MPS2]);
      logger->log("bias/az_mps2", bias[TConfig::INPUT_AZ_MPS2]);
    }
  }

private:
  /**
   * @brief sets the all calculated offsets in the imu_handler
   */
  void update_imu_offsets(void)
  {
    const double abs_velocity = std::hypot(x_out_[TConfig::STATE_VX_MPS], x_out_[TConfig::STATE_VY_MPS]);
    imu_handler_->update_sensor_bias(abs_velocity < 0.4);
    if constexpr (!TConfig::THREE_DIMENSIONAL) {
      // set the external orientation prior for the imu handler
      imu_handler_->set_external_orientation(external_orientation_);
    }
  }

  /**
   * @brief Declare and update the parameters of the state estimation
   */
  void declare_and_update_parameters(void)
  {
    // clang-format off
    state_estimation_params_.initialize_from_measurements = param_manager_->declare_and_get_value("enable.initialize_from_measurements", true, tam::pmg::ParameterType::BOOL, "Force the state estimation to initialize its state using sensor measurements").as_bool(); // NOLINT
    state_estimation_params_.fuse_external_orientation = param_manager_->declare_and_get_value("enable.external_orientation_measurement", false, tam::pmg::ParameterType::BOOL, "Allow the state estimation to fuse the external orientation prior as orientation measurement").as_bool(); // NOLINT
    state_estimation_params_.external_orientation_R_init = param_manager_->declare_and_get_value("external_orientation.orientation.R_init", std::vector<double>{0.1, 0.1}, tam::pmg::ParameterType::DOUBLE_ARRAY, "Constant Measurement Noise Covariance for the external orientation prior").as_double_array(); // NOLINT
    state_estimation_params_.external_orientation_outlier_bounds = param_manager_->declare_and_get_value("external_orientation.orientation.outlier_bounds", std::vector<double>{0.1, 0.1}, tam::pmg::ParameterType::DOUBLE_ARRAY, "Constant Outlier Bounds for the external orientation prior").as_double_array(); // NOLINT
    state_estimation_params_.fuse_ref_angles = param_manager_->declare_and_get_value("enable.reference_angle_measurement", true, tam::pmg::ParameterType::BOOL, "Allow the state estimation to fuse the reference orientation as orientation measurement").as_bool(); // NOLINT
    state_estimation_params_.overwrite_pose_covariance_threshold = param_manager_->declare_and_get_value("state_machine.overwrite_pose_covariance_threshold", 0.5, tam::pmg::ParameterType::DOUBLE, "Allow the state estimation to overwrite the state machine if the state covariance for the position is below this threshold").as_double(); // NOLINT
    state_estimation_params_.hard_linear_velocity_outlier_th = param_manager_->declare_and_get_value("hard_outlier_rejection.treshold.linear_velocity_mps", 10.0, tam::pmg::ParameterType::DOUBLE, "Squared distance threshold used to set the linear velocity input invalid").as_double(); // NOLINT
    state_estimation_params_.hard_accelerometer_outlier_th = param_manager_->declare_and_get_value("hard_outlier_rejection.treshold.accelerometer_mps2", 1000.0, tam::pmg::ParameterType::DOUBLE, "Squared distance threshold on acceleration input used to set the IMU input invalid").as_double(); // NOLINT
    state_estimation_params_.hard_angular_velocity_outlier_th = param_manager_->declare_and_get_value("hard_outlier_rejection.treshold.angular_velocity_radps", 1.0, tam::pmg::ParameterType::DOUBLE, "Squared distance threshold on angular velocity input used to set the IMU input invalid").as_double(); // NOLINT
    state_estimation_params_.max_consecutive_vel_hard_outliers = param_manager_->declare_and_get_value("hard_outlier_rejection.max_consecutive_outlier.linear_velocity", 250, tam::pmg::ParameterType::INTEGER, "Number of Consecutive hard outliers before changing the State Machine status").as_int(); // NOLINT
    state_estimation_params_.max_consecutive_imu_hard_outliers = param_manager_->declare_and_get_value("hard_outlier_rejection.max_consecutive_outlier.imu", 50, tam::pmg::ParameterType::INTEGER, "Number of Consecutive hard outliers before changing the State Machine status").as_int(); // NOLINT
    state_estimation_params_.u_bias_init = param_manager_->declare_and_get_value("inputs.bias_init", std::vector<double>{0.0, 0.0, 0.0}, tam::pmg::ParameterType::DOUBLE_ARRAY, "Initial Bias for the angular velocities").as_double_array(); // NOLINT
    // clang-format on
    previous_param_state_hash_ = param_manager_->get_state_hash();
  }

private:
  /**
   * @brief TAM Parameter Manager Composer to combine the parameters of all subclasses
   */
  tam::pmg::ParamManagerComposer::SharedPtr param_manager_composer_{};

  /**
   * @brief TAM Parameter Manager for the parameter management of the state estimation
   */
  tam::pmg::ParamValueManager::SharedPtr param_manager_ = std::make_shared<tam::pmg::ParamValueManager>();

  /**
   * @brief Variable containing the previous parameter state hash
   */
  std::size_t previous_param_state_hash_{0};

  /**
   * @brief Struct containing the parameters for the State Estimation
   */
  tam::types::state::config::state_estimation state_estimation_params_;

  /**
   * @brief Vector storring a mapping between the measurement identifier and name
   */
  std::vector<std::pair<tam::types::state::measurements::identifier, std::string>> identifier_{};

  /**
   * @brief State Machine of the State Estimation
   */
  std::shared_ptr<tam::core::state::StateMachine<TConfig>> state_machine_{};

  /**
   * @brief Vehicle model class to model the tires and calculate the
   *        linear velocity input for the State Estimation
   */
  std::shared_ptr<tam::core::state::VehicleModelHandler<TConfig, TModel>> vehicle_model_handler_{};

  /**
   * @brief IMU preprocessing to fuse the IMU measurements and compensate for sensor biases
   */
  std::shared_ptr<tam::core::state::IMUHandler<TConfig>> imu_handler_{};

  /**
   * @brief Reference orientation handler to calculate pitch and roll based on the imu measurements
   *        and the predicted vehicle odometry
   */
  std::shared_ptr<tam::core::state::RefOrientationHandler<TConfig>> ref_orientation_handler_{};

  /**
   * @brief Kalman Filter of the State Estimation
   */
  std::shared_ptr<tam::core::state::KFBase<TConfig>> kalman_filter_{};

  /**
   * @brief Input vector defined in the template structs in:
   *        lib_cpp/constants/state_estimation_constants
   */
  Eigen::Vector<double, TConfig::INPUT_VECTOR_SIZE> u_{};

  /**
   * @brief Raw IMU measurements of input vector defined in the template structs in:
   *        lib_cpp/constants/state_estimation_constants
   */
  // clang-format off
  Eigen::Vector<double, TConfig::INPUT_VECTOR_SIZE * (TConfig::NUM_IMU_MEASUREMENT + TConfig::NUM_BACKUP_IMU_MEASUREMENT)> u_raw_{};  // NOLINT
  // clang-format on

  /**
   * @brief Raw IMU covariance of input vector defined in the template structs in:
   *        lib_cpp/constants/state_estimation_constants
   */
  // clang-format off
  Eigen::Vector<double, TConfig::INPUT_VECTOR_SIZE * (TConfig::NUM_IMU_MEASUREMENT + TConfig::NUM_BACKUP_IMU_MEASUREMENT)> u_cov_{};  // NOLINT
  // clang-format on

  /**
   * @brief Measurement vector defined in the template structs in:
   *        lib_cpp/constants/state_estimation_constants
   */
  Eigen::Vector<double, TConfig::MEASUREMENT_VECTOR_SIZE> z_{};

  /**
   * @brief Vector indicating which sensors have been updated
   *        (1: update, 0: not updated)
   */
  Eigen::Vector<double, TConfig::MEASUREMENT_VECTOR_SIZE> fusion_vec_{};

  /**
   * @brief State vector of the Kalman Filter to handle the outputs
   */
  Eigen::Vector<double, TConfig::STATE_VECTOR_SIZE> x_out_{};

  /**
   * @brief Vector representing the diagonal elements of the measurement covariance matrix to adapt
   */
  Eigen::Vector<double, TConfig::MEASUREMENT_VECTOR_SIZE> R_vec_adapt_{};

  /**
   * @brief Vector buffering the number of consecutive hard outliers of the state estimation inputs
   */
  // clang-format off
  Eigen::Vector<int, TConfig::NUM_VEL_MEASUREMENT + 2 * (TConfig::NUM_IMU_MEASUREMENT + TConfig::NUM_BACKUP_IMU_MEASUREMENT)> outlier_buffer_{};  // NOLINT
  // clang-format on

  /**
   * @brief Boolean array indicating whether the asynchronous IMU filter should be stepped
   */
  Eigen::Vector<int, (TConfig::NUM_IMU_MEASUREMENT + TConfig::NUM_BACKUP_IMU_MEASUREMENT)> updated_acceleration_{};
  Eigen::Vector<int, (TConfig::NUM_IMU_MEASUREMENT + TConfig::NUM_BACKUP_IMU_MEASUREMENT)> updated_angular_velocity_{};

  /**
   * @brief External orientation prior [banking, slope, 0.0] based on the banking map
   */
  tam::types::common::Vector3D<double> external_orientation_{0.0, 0.0, 0.0};

  /**
   * @brief Vector containing the track information based on the banking map
   */
  tam::types::common::Vector3D<double> ref_angles_{0.0, 0.0, 0.0};

  /**
   * @brief Vector containing the vehicle velocities calculated from the vehicle model
   */
  tam::types::common::Vector3D<double> vehicle_model_velocity_{0.0, 0.0, 0.0};
};
}  // namespace tam::core::state
