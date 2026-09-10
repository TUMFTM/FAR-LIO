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
#include "tum_types_cpp/common.hpp"
#include "tum_types_cpp/control.hpp"

// Param manager
#include "param_management_cpp/param_value_manager.hpp"

// State Estimation constants / template input
#include "state_estimation_constants/EKF_2D.hpp"
#include "state_estimation_constants/EKF_3D.hpp"

namespace tam::core::state {
template <class TConfig>
class StateMachine
{
public:
  /**
   * @brief Constructor
   */
  StateMachine()
  {
    // initialize the valid map to false
    for (uint8_t i = 0; i < TConfig::NUM_POS_MEASUREMENT; ++i)
      valid_map_[{tam::types::state::measurements::POS, i}] = false;
    for (uint8_t i = 0; i < TConfig::NUM_ORIENTATION_MEASUREMENT; ++i)
      valid_map_[{tam::types::state::measurements::ORIENTATION, i}] = false;
    for (uint8_t i = 0; i < TConfig::NUM_VEL_MEASUREMENT; ++i)
      valid_map_[{tam::types::state::measurements::VEL, i}] = false;
    for (uint8_t i = 0; i < TConfig::NUM_IMU_MEASUREMENT; ++i)
      valid_map_[{tam::types::state::measurements::IMU, i}] = false;
    this->declare_and_update_parameters();
  };

  /**
   * @brief Update the State Estimaion State Machine
   *
   * @param[in] overwrite_state_machine   - bool:
   *                                        this function input allows the overwrite the state machine logic
   *                                        such that all states can be reached.
   *
   * OK:        State Estimation functions as expected
   * WARN:      The qualitiy of the state estimation prediction is degraded (no emergency)
   * ERROR:     An important sensor signal is missing (execute a soft emergency stop on EKF)
   * STALE:     All safety critical sensors are missing (execute a hard emergency stop)
   *
   *                        |  OK  | STALE |  WARN  | STALE | ERROR | STALE | ERROR | STALE
   * ---------------------------------------------------------------------------------------
   * any valid_loc_x        |   x  |   x   |    x   |   x   |       |       |       |
   * ---------------------------------------------------------------------------------------
   * any valid_lin_vel_x    |   x  |   x   |        |       |   x   |   x   |       |
   * ---------------------------------------------------------------------------------------
   * any valid_imu_x        |   x  |       |    x   |       |   x   |       |   x   |
   *
   * In addition to the truth table we perform a safe stop if:
   * number of valid imus < P_VDC_MinValidIMUs || backup imu is the only active imu
   *
   */
  void update(bool overwrite_state_machine = false)
  {
    if (previous_param_state_hash_ != param_manager_->get_state_hash()) {
      this->declare_and_update_parameters();
    }
    // check all valid whether atleast one input source per modalitiy is valid
    bool valid_loc = false, valid_lin_vel = false, valid_imu = false;
    bool valid_pos = false, valid_orientation = false, imu_safe_stop = false;

    // check whether there is a valid position input
    for (uint8_t i = 0; i < TConfig::NUM_POS_MEASUREMENT; ++i)
      valid_pos |= valid_map_[{tam::types::state::measurements::POS, i}];

    // check whether there is a valid orientation input
    // dont consider the last two measurements because they are reserved for internal usage
    for (uint8_t i = 0; i < TConfig::NUM_ORIENTATION_MEASUREMENT - 2; ++i)
      valid_orientation |= valid_map_[{tam::types::state::measurements::ORIENTATION, i}];

    // combine the valid bits for position and orientation to obtain the a valid localization
    valid_loc = valid_orientation & valid_pos;

    // check whether there is a valid linear velocity input
    for (uint8_t i = 0; i < TConfig::NUM_VEL_MEASUREMENT; ++i)
      valid_lin_vel |= valid_map_[{tam::types::state::measurements::VEL, i}];

    // check whether there is a valid imu input
    for (uint8_t i = 0; i < (TConfig::NUM_IMU_MEASUREMENT + TConfig::NUM_BACKUP_IMU_MEASUREMENT); ++i)
      valid_imu |= valid_map_[{tam::types::state::measurements::IMU, i}];

    // perform a safe stop if the P_VDC_MinValidIMUs threshold is voilated or the backup imu mode
    // is activated (only active if one lin_vel and loc is valid)
    // clang-format off
    bool backup_imu_valid = valid_map_[{tam::types::state::measurements::IMU, TConfig::NUM_IMU_MEASUREMENT + TConfig::NUM_BACKUP_IMU_MEASUREMENT - 1}]; // NOLINT
    if (get_num_valid_imus() < state_machine_params_.min_valid_imus) imu_safe_stop = true;
    if (get_num_valid_imus() == 0 && backup_imu_active_ && backup_imu_valid) imu_safe_stop = true;
    // clang-format on

    // check whether more than the state estimation internal linear velocity input is valid
    // this indicates that a valid vehicle side slip can be estimated
    bool valid_side_slip = get_num_valid_vels() > 1;

    // overwrite_state_machine allows any state transition
    // this should only be used to initialize or recover the state machine
    if (overwrite_state_machine) {
      if (valid_loc & valid_imu & valid_lin_vel & valid_side_slip & !imu_safe_stop) {
        state_ = tam::types::ErrorLvl::OK;
        status_message_ = "OK";
      } else if (valid_loc & valid_imu & valid_lin_vel & !valid_side_slip & !imu_safe_stop) {
        state_ = tam::types::ErrorLvl::WARN;
        status_message_ = "no valid ssa";
      } else if (((!imu_safe_stop) | (imu_safe_stop & valid_lin_vel & valid_loc)) & valid_imu) {
        state_ = tam::types::ErrorLvl::ERROR;
        status_message_ = "no valid:";
        if (!valid_loc) status_message_ += " loc";
        if (!valid_lin_vel) status_message_ += " lin_vel";
        if (imu_safe_stop) status_message_ += " main_imu";
      } else {
        state_ = tam::types::ErrorLvl::STALE;
        status_message_ = "no valid imu";
      }
    } else {
      // set the state estimation status
      // if the state estimation status is OK or WARN all states can be reached
      if (state_ == tam::types::ErrorLvl::OK || state_ == tam::types::ErrorLvl::WARN) {
        if (valid_loc & valid_imu & valid_lin_vel & valid_side_slip & !imu_safe_stop) {
          state_ = tam::types::ErrorLvl::OK;
          status_message_ = "OK";
        } else if (valid_loc & valid_imu & valid_lin_vel & !valid_side_slip & !imu_safe_stop) {
          state_ = tam::types::ErrorLvl::WARN;
          status_message_ = "no valid ssa";
        } else if (((!imu_safe_stop) | (imu_safe_stop & valid_lin_vel & valid_loc)) & valid_imu) {
          state_ = tam::types::ErrorLvl::ERROR;
          status_message_ = "no valid:";
          if (!valid_loc) status_message_ += " loc";
          if (!valid_lin_vel) status_message_ += " lin_vel";
          if (imu_safe_stop) status_message_ += " main_imu";
        } else {
          state_ = tam::types::ErrorLvl::STALE;
          status_message_ = "no valid imu";
        }
        // if the state estimation is in state Error the Error
        // is buffered and the state can only get worse
      } else if (state_ == tam::types::ErrorLvl::ERROR) {
        if (((!imu_safe_stop) | (imu_safe_stop & valid_lin_vel & valid_loc)) & valid_imu) {
          state_ = tam::types::ErrorLvl::ERROR;
          status_message_ = "no valid:";
          if (!valid_loc) status_message_ += " loc";
          if (!valid_lin_vel) status_message_ += " lin_vel";
          if (imu_safe_stop) status_message_ += " main_imu";
          if (valid_loc && valid_lin_vel && !imu_safe_stop) status_message_ = "error buffered";
        } else {
          state_ = tam::types::ErrorLvl::STALE;
          status_message_ = "no valid imu";
        }
        // if the state estimation is already in STALE it can only stay in STALE
      } else {
        state_ = tam::types::ErrorLvl::STALE;
        if (!valid_imu) {
          status_message_ = "no valid imu";
        } else {
          status_message_ = "stale buffered";
        }
      }
    }
  }

  /**
   * @brief forwards the status signal of the position input to the state estimation state machine
   *
   * @param[in] status                    - tam::types::ErrorLvl:
   *                                        containing information on the status of the position input
   *                                        (0: OK, 1: Warn, 2: Error, 3: STALE)
   * @param[in] pos_num                   - uint8_t:
   *                                        containing the number of the position input [0-N]
   */
  void set_position_status(const tam::types::ErrorLvl& status, uint8_t pos_num)
  {
    // set the position input valid if the sensor status is OK (0)
    if (pos_num >= TConfig::NUM_POS_MEASUREMENT) return;
    if (status == tam::types::ErrorLvl::OK || status == tam::types::ErrorLvl::WARN) {
      valid_map_[{tam::types::state::measurements::POS, pos_num}] = true;
    } else {
      valid_map_[{tam::types::state::measurements::POS, pos_num}] = false;
    }

    // update the state machine
    update();
  };

  /**
   * @brief forwards the status signal of the orientation input to the state estimation state machine
   *
   * @param[in] status                    - tam::types::ErrorLvl:
   *                                        containing information on the status of the orientation input
   *                                        (0: OK, 1: Warn, 2: Error, 3: STALE)
   * @param[in] orientation_num           - uint8_t:
   *                                        containing the number of the orientation input [0-N]
   */
  void set_orientation_status(const tam::types::ErrorLvl& status, uint8_t orientation_num)
  {
    // set the position input valid if the sensor status is OK (0)
    if (orientation_num >= TConfig::NUM_ORIENTATION_MEASUREMENT) return;
    if (status == tam::types::ErrorLvl::OK || status == tam::types::ErrorLvl::WARN) {
      valid_map_[{tam::types::state::measurements::ORIENTATION, orientation_num}] = true;
    } else {
      valid_map_[{tam::types::state::measurements::ORIENTATION, orientation_num}] = false;
    }

    // update the state machine
    update();
  };

  /**
   * @brief forwards the status signal of the linear velocity input to the state estimation state machine
   *
   * @param[in] status                    - tam::types::ErrorLvl:
   *                                        containing information on the status of the localization input
   *                                        (0: OK, 1: Warn, 2: Error, 3: STALE)
   * @param[in] vel_num                   - uint8_t:
   *                                        containing the number of the linear velocity input [0-N]
   */
  void set_linear_velocity_status(const tam::types::ErrorLvl& status, uint8_t vel_num)
  {
    // set the linear velocity input valid if the sensor status is OK (0)
    if (vel_num >= TConfig::NUM_VEL_MEASUREMENT) return;
    if (status == tam::types::ErrorLvl::OK) {
      valid_map_[{tam::types::state::measurements::VEL, vel_num}] = true;
    } else {
      valid_map_[{tam::types::state::measurements::VEL, vel_num}] = false;
    }

    // update the state machine
    update();
  };

  /**
   * @brief forwards the status signal of the imu input to the state estimation state machine
   *
   * @param[in] status                    - tam::types::ErrorLvl:
   *                                        containing information on the status of the localization input
   *                                        (0: OK, 1: Warn, 2: Error, 3: STALE)
   * @param[in] imu_num                   - uint8_t:
   *                                        containing the number of the imu input [0-N]
   */
  void set_imu_status(const tam::types::ErrorLvl& status, uint8_t imu_num)
  {
    // set the imu input valid if the sensor status is OK (0)
    if (imu_num >= TConfig::NUM_IMU_MEASUREMENT + TConfig::NUM_BACKUP_IMU_MEASUREMENT) return;
    if (status == tam::types::ErrorLvl::OK) {
      valid_map_[{tam::types::state::measurements::IMU, imu_num}] = true;
      if (imu_num < TConfig::NUM_IMU_MEASUREMENT) set_backup_imu_inactive();
    } else {
      // handle backup imu
      bool prev_status = valid_map_[{tam::types::state::measurements::IMU, imu_num}];
      if (imu_num < TConfig::NUM_IMU_MEASUREMENT && get_num_valid_imus() == 1 && prev_status) {
        set_backup_imu_active();
      }
      valid_map_[{tam::types::state::measurements::IMU, imu_num}] = false;
    }

    // update the state machine
    update();
  };

  /**
   * @brief sets the position input in the state estimation state machine invalid
   *        (signal is not longer fused, and state machine is updated)
   *
   * @param[in] pos_num                   - uint8_t:
   *                                        containing the number of the position input [0-N]
   */
  void set_position_invalid(uint8_t pos_num)
  {
    // set the position input invalid
    if (pos_num >= TConfig::NUM_POS_MEASUREMENT) return;
    valid_map_[{tam::types::state::measurements::POS, pos_num}] = false;

    // update the state machine
    update();
  };

  /**
   * @brief sets the orientation input in the state estimation state machine invalid
   *        (signal is not longer fused, and state machine is updated)
   *
   * @param[in] orientation_num           - uint8_t:
   *                                        containing the number of the orientation input [0-N]
   */
  void set_orientation_invalid(uint8_t orientation_num)
  {
    // set the position input invalid
    if (orientation_num >= TConfig::NUM_ORIENTATION_MEASUREMENT) return;
    valid_map_[{tam::types::state::measurements::ORIENTATION, orientation_num}] = false;

    // update the state machine
    update();
  };

  /**
   * @brief sets the linear velocity input in the state estimation state machine invalid
   *        (signal is not longer fused, and state machine is updated)
   *
   * @param[in] vel_num                   - uint8_t:
   *                                        containing the number of the linear velocity input [0-N]
   */
  void set_linear_velocity_invalid(uint8_t vel_num)
  {
    // set the linear velocity input invalid
    if (vel_num >= TConfig::NUM_VEL_MEASUREMENT) return;
    valid_map_[{tam::types::state::measurements::VEL, vel_num}] = false;

    // update the state machine
    update();
  };

  /**
   * @brief sets the imu input in the state estimation state machine invalid
   *        (signal is not longer fused, and state machine is updated)
   *
   * @param[in] imu_num                   - uint8_t:
   *                                        containing the number of the imu input [0-N]
   */
  void set_imu_invalid(uint8_t imu_num)
  {
    // set the imu input invalid
    if (imu_num >= TConfig::NUM_IMU_MEASUREMENT + TConfig::NUM_BACKUP_IMU_MEASUREMENT) return;
    bool prev_status = valid_map_[{tam::types::state::measurements::IMU, imu_num}];
    if (imu_num < TConfig::NUM_IMU_MEASUREMENT && get_num_valid_imus() == 1 && prev_status) {
      set_backup_imu_active();
    }
    valid_map_[{tam::types::state::measurements::IMU, imu_num}] = false;

    // update the state machine
    update();
  };

  /**
   * @brief sets the backup imu of the car active to perform a safe stop
   */
  void set_backup_imu_active(void)
  {
    // active the backup imu to perform a safe stop on this one imu if it is valid
    uint8_t backup_imu_num = TConfig::NUM_IMU_MEASUREMENT + TConfig::NUM_BACKUP_IMU_MEASUREMENT - 1;
    if (valid_map_[{tam::types::state::measurements::IMU, backup_imu_num}]) {
      backup_imu_active_ = true;
    }

    // update the state machine
    update();
  };

  /**
   * @brief sets the backup imu of the car inactive
   */
  void set_backup_imu_inactive(void)
  {
    // deactivate the backup imu
    backup_imu_active_ = false;

    // update the state machine
    update();
  };

  /**
   * @brief Get the current State of the State Machine
   *
   * @param[out]                          - tam::types::ErrorLvl:
   *                                        current state of the state estimation
   */
  tam::types::ErrorLvl get_state(void) const { return state_; };

  /**
   * @brief Get a vector of the size of the measurement vector indicating which signal is valid
   *
   * @param[out]                          - Eigen::VectorXd:
   *                                        fusion vector which signal should be fused
   */
  Eigen::Vector<double, TConfig::MEASUREMENT_VECTOR_SIZE> get_valid_fusion_vec(void)
  {
    Eigen::Vector<double, TConfig::MEASUREMENT_VECTOR_SIZE> valid_fusion_vec;
    valid_fusion_vec.setZero();

    // check all position valid bits
    // clang-format off
    for (uint8_t i = 0; i < TConfig::NUM_POS_MEASUREMENT; ++i) {
      if (valid_map_[{tam::types::state::measurements::POS, i}]) {
        valid_fusion_vec.segment(i * TConfig::POS_MEASUREMENT_VECTOR_SIZE, TConfig::POS_MEASUREMENT_VECTOR_SIZE).setOnes(); // NOLINT
      }
    }
    // clang-format on

    // check all orientation valid bits
    // clang-format off
    for (uint8_t i = 0; i < TConfig::NUM_ORIENTATION_MEASUREMENT; ++i) {
      if (valid_map_[{tam::types::state::measurements::ORIENTATION, i}]) {
        valid_fusion_vec.segment(TConfig::MEASUREMENT_VECTOR_OFFSET_ORIENTATION + i * TConfig::ORIENTATION_MEASUREMENT_VECTOR_SIZE, TConfig::ORIENTATION_MEASUREMENT_VECTOR_SIZE).setOnes(); // NOLINT
      }
    }
    // clang-format on

    // check linear velocity valid bits
    // clang-format off
    for (uint8_t i = 0; i < TConfig::NUM_VEL_MEASUREMENT; ++i) {
      if (valid_map_[{tam::types::state::measurements::VEL, i}]) {
        valid_fusion_vec.segment(TConfig::MEASUREMENT_VECTOR_OFFSET_VEL + i * TConfig::VEL_MEASUREMENT_VECTOR_SIZE, TConfig::VEL_MEASUREMENT_VECTOR_SIZE).setOnes(); // NOLINT
      }
    }
    // clang-format on

    return valid_fusion_vec;
  };

  /**
   * @brief Get a vector of the size of the raw u (imu input) vector indicating which signal is valid
   *
   * @param[out]                          - Eigen::VectorXd:
   *                                        fusion vector which signal should be fused
   */
  // clang-format off
  const Eigen::Vector<double, TConfig::INPUT_VECTOR_SIZE*(TConfig::NUM_IMU_MEASUREMENT + TConfig::NUM_BACKUP_IMU_MEASUREMENT)> get_valid_u_fusion_vec(void) // NOLINT
  // clang-format on
  {
    // clang-format off
    Eigen::Vector<double, TConfig::INPUT_VECTOR_SIZE*(TConfig::NUM_IMU_MEASUREMENT + TConfig::NUM_BACKUP_IMU_MEASUREMENT)> valid_u_fusion_vec; // NOLINT
    valid_u_fusion_vec.setZero();

    // check imu valid bits only use the standard imus if the backup imu mode is not set active
    uint8_t num_used_imu_measurement = TConfig::NUM_IMU_MEASUREMENT;
    if (backup_imu_active_) num_used_imu_measurement += TConfig::NUM_BACKUP_IMU_MEASUREMENT;

    for (uint8_t i = 0; i < num_used_imu_measurement; ++i) {
      if (valid_map_[{tam::types::state::measurements::IMU, i}]) {
        valid_u_fusion_vec.segment(i * TConfig::INPUT_VECTOR_SIZE, TConfig::INPUT_VECTOR_SIZE).setOnes(); // NOLINT
      }
    }
    // clang-format on

    return valid_u_fusion_vec;
  };

  /**
   * @brief Resturns the number of currently valid imus
   *
   * @param[out]                          - uint8_t:
   *                                        number of valid imus
   */
  uint8_t get_num_valid_imus(void)
  {
    // check how many imus are valid
    uint8_t num_valid_imus = 0;
    for (uint8_t i = 0; i < TConfig::NUM_IMU_MEASUREMENT; ++i) {
      if (valid_map_[{tam::types::state::measurements::IMU, i}]) num_valid_imus += 1;
    }
    return num_valid_imus;
  };

  /**
   * @brief Returns the number of currently valid linear velocity measurement
   *
   * @param[out]                          - uint8_t:
   *                                        number of valid linear velocity measurement
   */
  uint8_t get_num_valid_vels(void)
  {
    // check how many imus are valid
    uint8_t num_valid_vels = 0;
    for (uint8_t i = 0; i < TConfig::NUM_VEL_MEASUREMENT; ++i) {
      if (valid_map_[{tam::types::state::measurements::VEL, i}]) num_valid_vels += 1;
    }
    return num_valid_vels;
  };

  /**
   * @brief Resturns a string containing the current status message of the state
   *
   * @param[out]                          - std::string
   */
  std::string get_status_msg(void) const { return status_message_; };

  /**
   * @brief get all individual valid bits for the state estimation output
   *
   * @param[out]                          - std::unordered_map:
   *                                        entire valid_map_ containing the sensor valid bits
   */
  const tam::types::state::unordered_identifier_map<bool>& get_debug(void) const { return valid_map_; };

  /**
   * @brief get a boolean indicating whether enough valid imus are available
   *
   * @param[in] min_valid_imus            - uint8_t:
   *                                        minimum number of valid imus
   */
  bool get_min_valid_imus(void) { return get_num_valid_imus() < state_machine_params_.min_valid_imus; };

  /**
   * @brief returns a pointer to the param manager
   *
   * @param[out]                          - std::shared_ptr<tam::interfaces::ParamManagerBase>
   */
  tam::pmg::ParamValueManager::SharedPtr get_param_handler(void) { return param_manager_; };

private:
  /**
   * @brief Declare and update the parameters of the State Machine
   */
  void declare_and_update_parameters(void)
  {
    // clang-format off
    state_machine_params_.min_valid_imus = param_manager_->declare_and_get_value("state_machine.min_valid_imus", 2, tam::pmg::ParameterType::INTEGER, "Number of minimum valid IMUs to not trigger an error").as_int(); // NOLINT
    // clang-format on
    previous_param_state_hash_ = param_manager_->get_state_hash();
  };

private:
  // Variables
  /**
   * @brief Status of the State Estimation
   *        (0: OK, 1: Warn, 2: Error, 3: STALE)
   */
  tam::types::ErrorLvl state_{};

  /**
   * @brief Vector containing all the sensor biases for the imu signals
   */
  tam::types::state::unordered_identifier_map<bool> valid_map_{};

  /**
   * @brief Status message of the state machine describing the current state
   */
  std::string status_message_{};

  /**
   * @brief Indicate whether the backup imu is active
   */
  bool backup_imu_active_{false};

  /**
   * @brief TAM Parameter Manager
   */
  tam::pmg::ParamValueManager::SharedPtr param_manager_ = std::make_shared<tam::pmg::ParamValueManager>();

  /**
   * @brief Variable containing the previous parameter state hash
   */
  std::size_t previous_param_state_hash_{0};

  /**
   * @brief Struct containing the parameters for the State Machine
   */
  tam::types::state::config::state_machine state_machine_params_{};
};
}  // namespace tam::core::state
