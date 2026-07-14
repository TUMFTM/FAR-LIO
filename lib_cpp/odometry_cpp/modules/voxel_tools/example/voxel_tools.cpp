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

#include "voxel_tools/voxel_tools.hpp"

#include <iostream>
#include <vector>

#include "odometry_types/odometry_config.hpp"
#include "odometry_types/point_types.hpp"
int main()
{
  // Define a frame of points
  tam::core::state::types::Point<tam::core::state::types::ICP_EXT> point_1;
  point_1.pos = Eigen::Vector3f(5.1, 2.1, 3.1);
  tam::core::state::types::Point<tam::core::state::types::ICP_EXT> point_2;
  point_2.pos = Eigen::Vector3f(5.8, 2.5, 3.9);
  tam::core::state::types::Point<tam::core::state::types::ICP_EXT> point_3;
  point_3.pos = Eigen::Vector3f(5.9, 2.9, 3.9);
  std::vector<tam::core::state::types::Point<tam::core::state::types::ICP_EXT>> frame{
    point_1, point_2, point_3};

  // Downsample the frame to a voxel grid and print the downsampled frame
  // clang-format off
  std::vector<tam::core::state::types::Point<tam::core::state::types::ICP_EXT>> frame_downsampled_cuda =  // NOLINT
      tam::core::state::voxel_downsample<tam::core::state::types::ICP_EXT>(frame, 1.0);
  // clang-format on

  // Check point to voxel function and print voxel
  tam::core::state::Voxel vox =
    tam::core::state::point_to_voxel<tam::core::state::types::ICP_EXT>(point_1, 1.0);
  // clang-format off
  std::cout << "Voxel of Point at (" << point_1.pos.x() << ", " << point_1.pos.y() << ", " << point_1.pos.z() << "): (" // NOLINT
            << vox.x() << ", " << vox.y() << ", " << vox.z() << ")" << std::endl;
  // clang-format on

  // Downsample the frame to a voxel grid and print the downsampled frame
  std::vector<tam::core::state::types::Point<tam::core::state::types::ICP_EXT>> frame_downsampled =
    tam::core::state::voxel_downsample<tam::core::state::types::ICP_EXT>(frame, 1.0);
  std::cout << "Original Frame:" << std::endl;
  for (const auto & point : frame) {
    // clang-format off
    std::cout << "Point at (" << point.pos.x() << ", " << point.pos.y() << ", " << point.pos.z() << ")" << std::endl; // NOLINT
    // clang-format on
  }
  std::cout << "Downsampled Frame:" << std::endl;
  for (const auto & point : frame_downsampled) {
    // clang-format off
    std::cout << "Point at (" << point.pos.x() << ", " << point.pos.y() << ", " << point.pos.z() << ")" << std::endl; // NOLINT
    // clang-format on
  }
  return 0;
}
