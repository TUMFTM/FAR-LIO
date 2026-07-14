/*
 * Copyright 2026 Maximilian Leitenstern, Marcel Weinmann
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
#include <map>
#include <memory>
#include <string>
#include <vector>

#include "odometry_types/odometry_types.hpp"
namespace tam::core::state::types
{
enum class InitStatus : std::uint8_t { WAITING_FOR_MAP = 0, WAITING_FOR_EKF = 1, READY = 2 };
/**
 * @brief Configuration for the node
 */
struct NodeConfig
{
  std::string odom_frame{};
  std::string child_frame{};
  std::string cloud_frame{};
  std::string input_pointcloud{};
  std::string input_pointcloud_status{};
  std::string output_odom{};
  std::string input_map{};
  std::string update_map_srv{};
};
/**
 * @brief Debug signals for the node
 */
struct NodeDebug
{
  bool map_update{false};
  double callback_time{0.0};
};
/**
 * @brief Configuration for the odometry node
 */
struct OdometryNodeConfig : public NodeConfig
{
  bool wait_tf{true};
};
/**
 * @brief Debug signals for the odometry node
 */
struct OdometryNodeDebug : public NodeDebug
{
};
}  // namespace tam::core::state::types
