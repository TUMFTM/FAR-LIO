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

#include "tam_odometry/tam_odometry.hpp"

#include "rclcpp_components/register_node_macro.hpp"

// Create the required default constructor to have a composable node
namespace tam_odometry {
struct ICP_EXT : public tam::core::state::OdometryNode<tam::core::state::types::ICP_EXT> {
  explicit ICP_EXT(const rclcpp::NodeOptions& options)
      : tam::core::state::OdometryNode<tam::core::state::types::ICP_EXT>(options)
  {
  }
};

struct ICP_CV : public tam::core::state::OdometryNode<tam::core::state::types::ICP_CV> {
  explicit ICP_CV(const rclcpp::NodeOptions& options)
      : tam::core::state::OdometryNode<tam::core::state::types::ICP_CV>(options)
  {
  }
};

struct GICP_EXT : public tam::core::state::OdometryNode<tam::core::state::types::GICP_EXT> {
  explicit GICP_EXT(const rclcpp::NodeOptions& options)
      : tam::core::state::OdometryNode<tam::core::state::types::GICP_EXT>(options)
  {
  }
};

#ifdef BUILD_CUDA
struct CUDA_ICP_EXT : public tam::core::state::OdometryNode<tam::core::state::types::CUDA_ICP_EXT> {
  explicit CUDA_ICP_EXT(const rclcpp::NodeOptions& options)
      : tam::core::state::OdometryNode<tam::core::state::types::CUDA_ICP_EXT>(options)
  {
  }
};

struct CUDA_GICP_EXT : public tam::core::state::OdometryNode<tam::core::state::types::CUDA_GICP_EXT> {
  explicit CUDA_GICP_EXT(const rclcpp::NodeOptions& options)
      : tam::core::state::OdometryNode<tam::core::state::types::CUDA_GICP_EXT>(options)
  {
  }
};

#endif
}  // namespace tam_odometry

RCLCPP_COMPONENTS_REGISTER_NODE(tam_odometry::ICP_EXT)          // NOLINT
RCLCPP_COMPONENTS_REGISTER_NODE(tam_odometry::ICP_CV)           // NOLINT
RCLCPP_COMPONENTS_REGISTER_NODE(tam_odometry::GICP_EXT)         // NOLINT
#ifdef BUILD_CUDA
RCLCPP_COMPONENTS_REGISTER_NODE(tam_odometry::CUDA_ICP_EXT)          // NOLINT
RCLCPP_COMPONENTS_REGISTER_NODE(tam_odometry::CUDA_GICP_EXT)         // NOLINT
#endif
