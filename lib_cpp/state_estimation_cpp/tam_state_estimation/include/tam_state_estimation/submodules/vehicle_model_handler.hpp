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

// TUM helpers
#include "tum_helpers_cpp/numerical.hpp"

// State Estimation constants / template input
#include "state_estimation_constants/EKF_2D.hpp"
#include "state_estimation_constants/EKF_3D.hpp"

namespace tam::core::state {
template <class TConfig, VehicleModel TModel>
class VehicleModelHandler
{
public:
  /**
   * @brief Constructor
   */
  VehicleModelHandler() { this->declare_and_update_parameters(); }

  /**
   * @brief Update the State Estimaion Wheelspeed Odometry input
   *
   * @param[in] wheel             - tam::types::common::DataPerWheel<double>:
   *                                angular velocity per wheel
   */
  void input_wheel_angular_velocities([[maybe_unused]] const tam::types::common::DataPerWheel<double>& wheel)
  {
    // calculate the average velocity over all wheels and apply the wheelspeed scale
    // if a vehicle model is configured
    if constexpr (TModel != VehicleModel::Kinematic) {
      dyn_tire_radii_scale_ = tam::helpers::numerical::interp(vehicle_odometry_.velocity_mps.x,
        vehicle_model_params_.rr_vel_scale_vel_points_mps, vehicle_model_params_.rr_vel_scale_scale_factors);

      const double average_absolute_velocity =
        ((wheel.front_left + wheel.front_right) * vehicle_model_params_.tyre_radius_front_m * dyn_tire_radii_scale_ +
          (wheel.rear_left + wheel.rear_right) * vehicle_model_params_.tyre_radius_rear_m * dyn_tire_radii_scale_) /
        4;

      // velocity in x- and y-direction calculated from the linear STM
      if constexpr (TModel == VehicleModel::SingleTrack) {
        vehicle_odometry_.velocity_mps.x = average_absolute_velocity * cos(side_slip_angle_);
        vehicle_odometry_.velocity_mps.y = average_absolute_velocity * sin(side_slip_angle_);
        vehicle_odometry_.velocity_mps.z = 0.0;
      } else {
        // average velocity over all wheels in x-direction
        vehicle_odometry_.velocity_mps.x = average_absolute_velocity;
        vehicle_odometry_.velocity_mps.y = 0.0;
        vehicle_odometry_.velocity_mps.z = 0.0;
      }
    }
  };

  /**
   * @brief Update the status of the wheelspeed signal
   *
   * @param[in] status            - tam::types::ErrorLvl:
   *                                Wheelspeed status
   */
  void input_wheelspeed_status(const tam::types::ErrorLvl status) { wheelspeed_status_ = status; };

  /**
   * @brief Update the State Estimaion Wheelspeed Odometry input
   *
   * @param[in] steering_angle    - double:
   *                                steering angle in rad
   */
  void input_steering_angle([[maybe_unused]] double steering_angle)
  {
    // update the side slip angle using a linear STM
    if constexpr (TModel == VehicleModel::SingleTrack) {
      side_slip_angle_ = std::atan(std::tan(steering_angle) * vehicle_model_params_.l_WheelbaseR_m /
        (vehicle_model_params_.l_WheelbaseF_m + vehicle_model_params_.l_WheelbaseR_m));
    }
  };

  /**
   * @brief Update the status of the steering angle status signal
   *
   * @param[in] status            - tam::types::ErrorLvl:
   *                                Wheelspeed status
   */
  void input_steering_angle_status(const tam::types::ErrorLvl status) { steering_angle_status_ = status; };

  /**
   * @brief Get the current linear velocities calculated form the angular velocities of the wheels
   *
   * @param[out] result           - tam::types::control::Odometry:
   *                                [vx_VelCoG_mps, vy_VelCoG_mps]
   */
  const tam::types::control::Odometry& get_vehicle_model_odometry(void) const { return vehicle_odometry_; };

  /**
   * @brief Get the current sensor status w.r.t. the vehicle model chosen
   *
   * @param[out] result           - tam::types::ErrorLvl:
   */
  tam::types::ErrorLvl get_status(void) const
  {
    // only return OK if the output of the vehicle model should be fused
    // and all needed sensors are OK
    if constexpr (TModel != VehicleModel::Kinematic) {
      if (wheelspeed_status_ != tam::types::ErrorLvl::OK) return tam::types::ErrorLvl::ERROR;
      if constexpr (TModel == VehicleModel::SingleTrack) {
        if (steering_angle_status_ != tam::types::ErrorLvl::OK) return tam::types::ErrorLvl::ERROR;
      }

      return tam::types::ErrorLvl::OK;
    }

    // else return an error such that the solution is not fused
    return tam::types::ErrorLvl::ERROR;
  };

  /**
   * @brief returns a pointer to the param manager
   *
   * @param[out]                  - std::shared_ptr<tam::interfaces::ParamManagerBase>
   */
  tam::pmg::ParamValueManager::SharedPtr get_param_handler(void) { return param_manager_; };

  /**
   * @brief Get the calculated tire radii scaling factor
   *
   * @param[out] result           - std::double
   */
  const double& get_dyn_tire_radii_scale(void) const { return dyn_tire_radii_scale_; };

  /**
   * @brief Declare and update the parameters of the vehicle model
   */
  void declare_and_update_parameters(void)
  {
    // clang-format off
    vehicle_model_params_.tyre_radius_front_m = param_manager_->declare_and_get_value("vehicle_model.parameter.tyreradius_front_m", 0.30, tam::pmg::ParameterType::DOUBLE, "Static front tyre radius").as_double(); // NOLINT
    vehicle_model_params_.tyre_radius_rear_m = param_manager_->declare_and_get_value("vehicle_model.parameter.tyreradius_rear_m", 0.30, tam::pmg::ParameterType::DOUBLE, "Static rear tyre radius").as_double(); // NOLINT
    vehicle_model_params_.l_WheelbaseF_m = param_manager_->declare_and_get_value("vehicle_model.parameter.l_WheelbaseF_m", 1.5, tam::pmg::ParameterType::DOUBLE, "Wheelbase of the car front").as_double(); // NOLINT
    vehicle_model_params_.l_WheelbaseR_m = param_manager_->declare_and_get_value("vehicle_model.parameter.l_WheelbaseR_m", 1.5, tam::pmg::ParameterType::DOUBLE, "Wheelbase of the car rear").as_double(); // NOLINT
    vehicle_model_params_.rr_vel_scale_vel_points_mps = param_manager_->declare_and_get_value("vehicle_model.parameter.rr_vel_scale_vel_points_mps", std::vector<double>({0, 100}), tam::pmg::ParameterType::DOUBLE_ARRAY, "").as_double_array(); // NOLINT
    vehicle_model_params_.rr_vel_scale_scale_factors = param_manager_->declare_and_get_value("vehicle_model.parameter.rr_vel_scale_scale_factors", std::vector<double>({1.0, 1.0}), tam::pmg::ParameterType::DOUBLE_ARRAY, "").as_double_array(); // NOLINT
    vehicle_model_params_.R_init = param_manager_->declare_and_get_value("vehicle_model.linear_velocity_mps.R_init", std::vector<double>{0.1, 0.1}, tam::pmg::ParameterType::DOUBLE_ARRAY, "Constant Measurement Noise Covariance for the computed linear velocity").as_double_array(); // NOLINT
    vehicle_model_params_.outlier_bounds = param_manager_->declare_and_get_value("vehicle_model.linear_velocity_mps.outlier_bounds", std::vector<double>{0.1, 0.1}, tam::pmg::ParameterType::DOUBLE_ARRAY, "Constant Outlier Bounds for the computed linear velocity").as_double_array(); // NOLINT
    // clang-format on
    previous_param_state_hash_ = param_manager_->get_state_hash();
  };

private:
  /**
   * @brief calculated vehicle velocity in the body frame
   */
  tam::types::control::Odometry vehicle_odometry_{};

  /**
   * @brief sensor status buffer for wheelspeed and steering angle
   */
  tam::types::ErrorLvl wheelspeed_status_{tam::types::ErrorLvl::STALE};
  tam::types::ErrorLvl steering_angle_status_{tam::types::ErrorLvl::STALE};

  /**
   * @brief side slip angle calculated from the vehicle model
   */
  double side_slip_angle_{0.0};

  /**
   * @brief tire radii scaling factor calculated based on vehicle velocity
   */
  double dyn_tire_radii_scale_{1.0};

  /**
   * @brief Variable containing the previous parameter state hash
   */
  std::size_t previous_param_state_hash_{0};

  /**
   * @brief Struct containing the parameters for the vehicle model
   */
  tam::types::state::config::vehicle_model vehicle_model_params_{};

  /**
   * @brief TAM Parameter Manager
   */
  tam::pmg::ParamValueManager::SharedPtr param_manager_ = std::make_shared<tam::pmg::ParamValueManager>();
};
}  // namespace tam::core::state
