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

#include <gtest/gtest.h>

#include <iostream>
#include <vector>

#include "odometry_types/odometry_config.hpp"
#include "odometry_types/point_types.hpp"
#include "voxel_tools/voxel_tools.hpp"
/**
 * @brief Test point to voxel conversion
 */
TEST(VoxelTools, PointToVoxel)
{
  // Define a frame of points
  tam::core::state::types::Point<tam::core::state::types::ICP_EXT> point_1;
  point_1.pos = Eigen::Vector3f(5.3, 2.5, 3.8);

  // Get Voxel of point
  tam::core::state::Voxel vox =
    tam::core::state::point_to_voxel<tam::core::state::types::ICP_EXT>(point_1, 1.0);

  EXPECT_EQ(vox, Eigen::Vector3i(5, 2, 3)) << "Failed to create voxel from point coordinates";
}
/**
 * @brief Test point to voxel conversion
 */
TEST(VoxelTools, VoxelDownsample)
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
  std::vector<tam::core::state::types::Point<tam::core::state::types::ICP_EXT>> frame_downsampled =
    tam::core::state::voxel_downsample<tam::core::state::types::ICP_EXT>(frame, 1.0);
  EXPECT_EQ(frame_downsampled.size(), 1) << "Wrong number of points in downsampled frame";
  EXPECT_EQ(frame_downsampled.front().pos, point_1.pos)
    << "Wrong point in downsampled frame";
}
