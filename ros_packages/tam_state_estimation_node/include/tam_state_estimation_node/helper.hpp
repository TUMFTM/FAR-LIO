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

#include <boost/algorithm/string.hpp>
#include <boost/range/iterator_range.hpp>
#include <cmath>
#include <iostream>
#include <stdexcept>
#include <map>
#include <rclcpp/rclcpp.hpp>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

#include "param_management_cpp/param_manager_composer.hpp"
#include "types.hpp"
#include "state_estimation_base/types.hpp"

namespace tam::core::state {
/**
 * @brief Parse the measurement and input sensor configuration from the ROS parameter overrides
 *
 * @param[in]  node               - the state estimation node (used to read/declare parameters)
 * @param[in]  param_manager_raw  - param manager to set the parsed IMU filter coefficients
 * @param[out] measurement_channels - parsed measurement sensor channels
 * @param[out] input_channels      - parsed IMU input channels
 * @param[out] identifier_names    - mapping from measurement identifier to sensor name for logging
 */
template <class TConfig>
void parse_sensor_configs(rclcpp::Node* node, tam::pmg::MgmtInterface* param_manager_raw,
  std::vector<tam::types::state::measurement>& measurement_channels,
  std::vector<tam::types::state::input>& input_channels,
  std::vector<std::pair<tam::types::state::measurements::identifier, std::string>>& identifier_names)
{
  std::unordered_map<std::string, std::map<std::string, rclcpp::ParameterValue>> measurement_map, input_map;
  const auto overrides = node->get_node_parameters_interface()->get_parameter_overrides();
  for (const auto& [name, value] : overrides) {
    std::vector<std::string> tokens;
    boost::split(tokens, name, boost::is_any_of("."));
    if (tokens.size() >= 3 && tokens[0] == "measurements") {
      const std::string key = boost::join(boost::make_iterator_range(tokens.begin() + 2, tokens.end()), ".");
      measurement_map[tokens[1]][key] = value;
      node->declare_parameter(name, value);
    } else if (tokens.size() >= 4 && tokens[0] == "inputs" && tokens[1] == "imus") {
      const std::string key = boost::join(boost::make_iterator_range(tokens.begin() + 3, tokens.end()), ".");
      input_map[tokens[2]][key] = value;
      node->declare_parameter(name, value);
    }
  }

  // Match ROS Overrides to Measurement Configuration Data Structure
  uint8_t num_position{}, num_orientation{}, num_linear_velocity{};
  for (const auto& [name, parameter] : measurement_map) {
    tam::types::state::measurement channel{};
    tam::types::state::config::measurement& cfg = channel.config;
    cfg.name = name;

    // Handling of false subscription config
    if (!parameter.count("message.topic"))
      throw std::invalid_argument(
        "[StateEstimationCPP]: Measurement input configuration for " + cfg.name + " does not provide a topic");
    if (!parameter.count("message.type"))
      throw std::invalid_argument(
        "[StateEstimationCPP]: Measurement input configuration for " + cfg.name + " does not provide a topic type");

    // Match Subscription config to data structure
    // clang-format off
    cfg.measurement_sub.topic = parameter.at("message.topic").template get<std::string>();
    cfg.measurement_sub.type = parameter.at("message.type").template get<std::string>();
    cfg.measurement_sub.timeout_ms = parameter.count("message.timeout_ms") ? parameter.at("message.timeout_ms").template get<int32_t>() : 500; // NOLINT
    cfg.measurement_sub.active = true;
    // clang-format on

    // Only nav_msgs/msg/Odometry measurement topics are currently supported
    if (cfg.measurement_sub.type != "nav_msgs/msg/Odometry")
      throw std::invalid_argument(
        "[StateEstimationCPP]: Measurement input configuration for " + cfg.name + " has an unimplemented topic type '"
        + cfg.measurement_sub.type + "', only 'nav_msgs/msg/Odometry' is supported");

    // Handling of status subscription config
    // clang-format off
    if (parameter.count("status.topic")) {
      cfg.status_sub.topic = parameter.at("status.topic").template get<std::string>();
      cfg.status_sub.timeout_ms = parameter.count("status.timeout_ms") ? parameter.at("status.timeout_ms").template get<int32_t>() : 500; // NOLINT
      cfg.status_sub.active = true;
    } else {
      std::cout << "[StateEstimationCPP]: Measurement input configuration for " + cfg.name + " does not provide a status topic, input is used without status" << std::endl; // NOLINT
      cfg.status_sub.active = false;
    }

    // Match additional delay on the topic
    cfg.additional_delay = parameter.count("message.additional_delay_ms") ? parameter.at("message.additional_delay_ms").template get<int32_t>() : 0; // NOLINT

    // Lamda to match fusion configs to the sensor
    auto setup_fusion_config = [&](const std::string& prefix, tam::types::state::config::fusion& fusion, uint8_t& num) { // NOLINT
      if (!parameter.count(prefix + ".R_init") || !parameter.count(prefix + ".outlier_bounds")) return;
      fusion.R_init = parameter.at(prefix + ".R_init").template get<std::vector<double>>();
      fusion.outlier_bounds = parameter.at(prefix + ".outlier_bounds").template get<std::vector<double>>();
      // a measurement-noise variance must be finite and strictly positive, otherwise the innovation
      // covariance S = H P H^T + R can become singular and S.inverse() produces NaNs in the update
      for (const double r : fusion.R_init)
        if (!std::isfinite(r) || r <= 0.0)
          throw std::invalid_argument("[StateEstimationCPP]: Measurement input configuration for " + cfg.name + " has a non-positive R_init entry for " + prefix); // NOLINT
      fusion.mask = parameter.count(prefix + ".mask") ? parameter.at(prefix + ".mask").template get<std::vector<bool>>() : std::vector<bool>(fusion.R_init.size(), true); // NOLINT
      fusion.num = num++;
      fusion.active = true;
      if (fusion.R_init.size() != fusion.outlier_bounds.size() || fusion.R_init.size() != fusion.mask.size())
        throw std::invalid_argument("[StateEstimationCPP]: Measurement input configuration for " + cfg.name + " does not provid consistent sizes for " + prefix); // NOLINT
    };

    // Match fusion configs for the measurements
    setup_fusion_config("position_m", cfg.position_m, num_position);
    setup_fusion_config("orientation_rad", cfg.orientation_rad, num_orientation);
    setup_fusion_config("linear_velocity_mps", cfg.linear_velocity_mps, num_linear_velocity);
    measurement_channels.push_back(channel);
    // clang-format on
  }

  // Match ROS Overrides to Input Configuration Data Structure
  uint8_t num_imu{};
  for (const auto& [name, parameter] : input_map) {
    tam::types::state::input channel{};
    tam::types::state::config::input& cfg = channel.config;
    cfg.name = name;

    // Handling of false subscription config
    if (!parameter.count("message.topic"))
      throw std::invalid_argument(
        "[StateEstimationCPP]: Input configuration for " + cfg.name + " does not provide a topic");
    if (!parameter.count("message.type"))
      throw std::invalid_argument(
        "[StateEstimationCPP]: Input configuration for " + cfg.name + " does not provide a topic type");

    // Match Subscription config to data structure
    // clang-format off
    cfg.measurement_sub.topic = parameter.at("message.topic").template get<std::string>();
    cfg.measurement_sub.type = parameter.at("message.type").template get<std::string>();
    cfg.measurement_sub.timeout_ms = parameter.count("message.timeout_ms") ? parameter.at("message.timeout_ms").template get<int32_t>() : 500; // NOLINT
    cfg.measurement_sub.active = true;

    // Only sensor_msgs/msg/Imu input topics are currently supported
    if (cfg.measurement_sub.type != "sensor_msgs/msg/Imu")
      throw std::invalid_argument(
        "[StateEstimationCPP]: Input configuration for " + cfg.name + " has an unimplemented topic type '"
        + cfg.measurement_sub.type + "', only 'sensor_msgs/msg/Imu' is supported");

    // Match IMU filter and backup settings
    cfg.filter_coefficients = parameter.count("filter_coefficients") ? parameter.at("filter_coefficients").template get<std::vector<double>>() : std::vector<double>({1.0}); // NOLINT
    cfg.backup = parameter.count("backup") ? parameter.at("backup").template get<bool>() : false;
    cfg.num = !cfg.backup ? num_imu++ : TConfig::NUM_IMU_MEASUREMENT + TConfig::NUM_BACKUP_IMU_MEASUREMENT - 1;
    identifier_names.emplace_back(tam::types::state::measurements::identifier{tam::types::state::measurements::IMU, cfg.num}, cfg.name); // NOLINT
    input_channels.push_back(channel);

    // Set IMU Filter Coefficients in the IMU handler
    param_manager_raw->set_value("imu_" + std::to_string(cfg.num + 1) + "_filter_coefficients", cfg.filter_coefficients); // NOLINT
    // clang-format on
    }
}
}  // namespace tam::core::state
