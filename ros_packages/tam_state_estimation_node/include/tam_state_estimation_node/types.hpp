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

// ROS
#include <rclcpp/time.hpp>
#include <tf2/LinearMath/Vector3.h>

namespace tam::types::state::config
{
  /**
   * @brief Struct containing the parameters for the state estimation node
   */
  struct node
  {
    double virtual_covariance_scale{};
    int32_t average_input_delay_ms{};
  };

  /**
   * @brief Struct containing configuration of a subscription
   */
  struct subscription
  {
    std::string topic{};
    std::string type{};
    bool active{false};
    int32_t timeout_ms{};
  };

  /**
   * @brief Struct containing the fusion parameters
   */
  struct fusion
  {
    std::vector<double> R_init{};
    std::vector<double> outlier_bounds{};
    std::vector<bool> mask{};
    uint8_t num{};
    bool active{false};
  };

  /**
   * @brief Struct containing the measurement configuration of the state estimation
   */
  struct measurement
  {
    std::string name{};
    tam::types::state::config::subscription measurement_sub{};
    tam::types::state::config::subscription status_sub{};
    tam::types::state::config::fusion position_m{};
    tam::types::state::config::fusion orientation_rad{};
    tam::types::state::config::fusion linear_velocity_mps{};
    int32_t additional_delay{};
  };

  /**
   * @brief Struct containing the input configuration of the state estimation
   */
  struct input
  {
    std::string name{};
    tam::types::state::config::subscription measurement_sub{};
    std::vector<double> filter_coefficients;
    uint8_t num{};
    bool backup{false};
  };
} // tam::types::state::config

namespace tam::types::state
{
  /**
   * @brief struct buffering the received odomety input and corresponding timestamp and config
   */
  struct measurement
  {
    tam::types::state::config::measurement config{};
    tam::types::control::Odometry odometry{};
    rclcpp::Time measurement_stamp{};
    rclcpp::Time received_stamp{};
    tf2::Vector3 static_translation{};
    bool static_translation_valid{false};
    bool received{false};
  };

  /**
   * @brief struct buffering the input config and received status
   */
  struct input
  {
    tam::types::state::config::input config{};
    bool received{false};
  };
} // tam::types::state
