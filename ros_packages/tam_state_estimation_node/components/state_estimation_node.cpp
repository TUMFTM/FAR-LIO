/*
 * Copyright 2024 Marcel Weinmann, Maximilian Leitenstern
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
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_components/register_node_macro.hpp>

#include "tam_state_estimation_node/node.hpp"

// Create the required default constructor to have a composable node
namespace tam_state_estimation_node {
struct StateEstimation2DEKFKin
    : public stateEstimationNode<tam::core::state::EKF_2D, tam::core::state::VehicleModel::Kinematic> {
  explicit StateEstimation2DEKFKin(const rclcpp::NodeOptions& options)
      : stateEstimationNode<tam::core::state::EKF_2D, tam::core::state::VehicleModel::Kinematic>(
          std::make_unique<
            tam::core::state::StateEstimation<tam::core::state::EKF_2D, tam::core::state::VehicleModel::Kinematic>>(),
          options)
  {
  }
};

struct StateEstimation2DEKFNh
    : public stateEstimationNode<tam::core::state::EKF_2D, tam::core::state::VehicleModel::NonHolonomic> {
  explicit StateEstimation2DEKFNh(const rclcpp::NodeOptions& options)
      : stateEstimationNode<tam::core::state::EKF_2D, tam::core::state::VehicleModel::NonHolonomic>(
          std::make_unique<tam::core::state::StateEstimation<tam::core::state::EKF_2D,
            tam::core::state::VehicleModel::NonHolonomic>>(),
          options)
  {
  }
};

struct StateEstimation3DEKFKin
    : public stateEstimationNode<tam::core::state::EKF_3D, tam::core::state::VehicleModel::Kinematic> {
  explicit StateEstimation3DEKFKin(const rclcpp::NodeOptions& options)
      : stateEstimationNode<tam::core::state::EKF_3D, tam::core::state::VehicleModel::Kinematic>(
          std::make_unique<
            tam::core::state::StateEstimation<tam::core::state::EKF_3D, tam::core::state::VehicleModel::Kinematic>>(),
          options)
  {
  }
};

struct StateEstimation3DEKFNh
    : public stateEstimationNode<tam::core::state::EKF_3D, tam::core::state::VehicleModel::NonHolonomic> {
  explicit StateEstimation3DEKFNh(const rclcpp::NodeOptions& options)
      : stateEstimationNode<tam::core::state::EKF_3D, tam::core::state::VehicleModel::NonHolonomic>(
          std::make_unique<tam::core::state::StateEstimation<tam::core::state::EKF_3D,
            tam::core::state::VehicleModel::NonHolonomic>>(),
          options)
  {
  }
};

struct StateEstimation3DEKFStm
    : public stateEstimationNode<tam::core::state::EKF_3D, tam::core::state::VehicleModel::SingleTrack> {
  explicit StateEstimation3DEKFStm(const rclcpp::NodeOptions& options)
      : stateEstimationNode<tam::core::state::EKF_3D, tam::core::state::VehicleModel::SingleTrack>(
          std::make_unique<
            tam::core::state::StateEstimation<tam::core::state::EKF_3D, tam::core::state::VehicleModel::SingleTrack>>(),
          options)
  {
  }
};
}  // namespace tam_state_estimation_node
RCLCPP_COMPONENTS_REGISTER_NODE(tam_state_estimation_node::StateEstimation2DEKFKin)
RCLCPP_COMPONENTS_REGISTER_NODE(tam_state_estimation_node::StateEstimation2DEKFNh)
RCLCPP_COMPONENTS_REGISTER_NODE(tam_state_estimation_node::StateEstimation3DEKFKin)
RCLCPP_COMPONENTS_REGISTER_NODE(tam_state_estimation_node::StateEstimation3DEKFNh)
RCLCPP_COMPONENTS_REGISTER_NODE(tam_state_estimation_node::StateEstimation3DEKFStm)
