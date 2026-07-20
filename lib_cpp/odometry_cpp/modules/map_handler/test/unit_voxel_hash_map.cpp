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

#include <algorithm>
#include <iostream>
#include <random>
#include <string>
#include <vector>

#include "test_utils.hpp"

/**
 * @brief Test construction of VoxelHashMap from param manager and logger
 */
TEST(VoxelHashMap, BuildPmgLogger)
{
  // clang-format off
  // Construct from param manager and logger
  tam::pmg::ParamReferenceManager::UniquePtr pmg_ =  std::make_unique<tam::pmg::ParamReferenceManager>(); // NOLINT
  tam::tsl::ReferenceLogger::UniquePtr logger_ = std::make_unique<tam::tsl::ReferenceLogger>(); // NOLINT
  auto map_ = tam::core::state::VoxelHashMap<tam::core::state::types::POINT_NORMAL>::from_config(pmg_.get(), logger_.get()); // NOLINT
  // clang-format on
  tam::pmg::MgmtInterface* pmg_raw = pmg_.get();
  pmg_raw->set_value("map.voxel_size", 5.0);

  EXPECT_EQ(map_->get_config().voxel_size, 5.0) << "Failed to construct VoxeHashMap from param manager and logger";
}

/**
 * @brief Test construction of VoxelHashMap from config and debug object
 */
TEST(VoxelHashMap, BuildConfigDebug)
{
  // clang-format off
  // Construct from config and debug objects
  tam::core::state::types::MapConfig config;
  tam::core::state::types::MapDebug debug;
  config.voxel_size = 1.0;
  auto map_ = tam::core::state::VoxelHashMap<tam::core::state::types::POINT_NORMAL>::from_config(config, debug); // NOLINT
  // clang-format on
  EXPECT_EQ(map_->get_config().voxel_size, 1.0) << "Failed to construct VoxeHashMap from config and debug object";
}

/**
 * @brief Check correct number of points after map initialization
 */
TEST(VoxelHashMap, NumPoints)
{
  auto map_ = init_map<tam::core::state::types::POINT_NORMAL>();
  EXPECT_EQ(map_->num_points(), 10) << "Wrong amount of points in map";
}

/**
 * @brief Check correct map size after adding points
 */
TEST(VoxelHashMap, AddPoints)
{
  // Generate random points in one voxel
  auto map_ = init_map<tam::core::state::types::POINT_NORMAL>(100);
  EXPECT_EQ(map_->num_points(), tam::core::state::types::POINT_NORMAL::MAX_POINTS_PER_VOXEL)
    << "Wrong amount of points per voxel";
}

/**
 * @brief Check correct correspondence calculation
 */
TEST(VoxelHashMap, SearchNeighbor)
{
  auto map_ = init_map<tam::core::state::types::POINT_NORMAL>();
  tam::core::state::types::Point<tam::core::state::types::POINT_NORMAL> query;
  query.pos = Eigen::Vector3f(5.05, 2.3, 3.15);

  const auto correspondence = map_->search_closest_neighbor(query);
  EXPECT_EQ(correspondence.map.pos, Eigen::Vector3f(5.1, 2.1, 3.1)) << "Wrong closest neighbor found!";
}

/**
 * @brief Check correct multiple nearest neighor search calculation
 */
TEST(VoxelHashMap, SearchNeighbors)
{
  auto map_ = init_map<tam::core::state::types::POINT_NORMAL>();
  std::vector<float> point_1_distances{0.000, 1.136, 1.386, 1.738, 1.349, 1.628, 1.158, 0.943, 1.449, 0.985};

  // Capture neigbors
  auto neighbors = map_->get_neighbors();

  EXPECT_EQ(neighbors.size(), 10) << "Wrong amount of neighbors in calculation!";
  EXPECT_EQ(neighbors.front().num_neighbors, tam::core::state::types::POINT_NORMAL::NUM_NEIGHBORS)
    << "Wrong amount of neighbors found!";
  // Check neighbors exemplary for first point
  for (const auto& neighbor : neighbors) {
    if ((neighbor.point->pos - Eigen::Vector3f(5.1, 2.1, 3.1)).norm() < 1e-5) {
      // Sort distances
      std::sort(point_1_distances.begin(), point_1_distances.end());
      std::array<float, tam::core::state::types::POINT_NORMAL::NUM_NEIGHBORS> distances_computed(
        std::to_array(neighbor.distance));
      std::sort(distances_computed.begin(), distances_computed.end());
      // Check distances for computed neighbors
      for (size_t i = 0; i < tam::core::state::types::POINT_NORMAL::NUM_NEIGHBORS; ++i) {
        EXPECT_NEAR(distances_computed[i], point_1_distances[i], 1e-3) << "Wrong neighbor for point 1!";
      }
      break;
    }
  }
}

/**
 * @brief Check correct normal computation for neighbors
 */
TEST(VoxelHashMap, Normals)
{
  auto map_ = init_map<tam::core::state::types::POINT_NORMAL>();

  // Capture neigbors
  auto neighbors = map_->get_neighbors();

  EXPECT_EQ(neighbors.size(), 10) << "Wrong amount of neighbors in calculation!";
  EXPECT_EQ(neighbors.front().num_neighbors, tam::core::state::types::POINT_NORMAL::NUM_NEIGHBORS)
    << "Wrong amount of neighbors found!";
  // Check normal exemplary for first point
  for (const auto& neighbor : neighbors) {
    if ((neighbor.point->pos - Eigen::Vector3f(5.1, 2.1, 3.1)).norm() < 1e-5) {
      // Check normals for computed neighbor
      // Compare results
      // clang-format off
      EXPECT_NEAR(neighbor.point->normal.x(), -0.561368f, 1e-5) << "Wrong normal (x coordinate)!";  // NOLINT
      EXPECT_NEAR(neighbor.point->normal.y(), 0.826427f, 1e-5) << "Wrong normal (y coordinate)!";  // NOLINT
      EXPECT_NEAR(neighbor.point->normal.z(), 0.0434006f, 1e-5) << "Wrong normal (z coordinate)!";
      // clang-format on
      break;
    }
  }
}

/**
 * @brief Check correct normal computation for neighbors
 */
TEST(VoxelHashMap, Covariances)
{
  // Test SVD regularization
  auto map_svd = init_map<tam::core::state::types::POINT_NORMAL>(10, "SVD");
  auto neighbors_svd = map_svd->get_neighbors();

  // Test FROBENIUS regularization
  auto map_frob = init_map<tam::core::state::types::POINT_NORMAL>(10, "FROBENIUS");
  auto neighbors_frob = map_frob->get_neighbors();

  // Test MIN_EIGENVALUE regularization
  auto map_min = init_map<tam::core::state::types::POINT_NORMAL>(10, "MIN_EIGENVALUE");
  auto neighbors_min = map_min->get_neighbors();

  Eigen::Matrix3f expected_cov_SVD, expected_cov_FROBENIUS, expected_cov_MIN_EIGENVALUE;
  expected_cov_SVD << 0.685181, 0.463466, 0.0243394, 0.463466, 0.317701, -0.0358316, 0.0243394, -0.0358316, 0.998118;
  expected_cov_FROBENIUS << 31.1507, 20.4267, 1.02138, 20.4267, 15.6264, -14.2987, 1.02138, -14.2987, 286.484;
  expected_cov_MIN_EIGENVALUE << 0.060013, 0.0399809, 0.00199909, 0.0399809, 0.0296282, -0.0279985, 0.00199909,
    -0.0279985, 0.560000;

  // Check for each regularization type
  for (const auto& neighbor : neighbors_svd) {
    if ((neighbor.point->pos - Eigen::Vector3f(5.1, 2.1, 3.1)).norm() < 1e-5) {
      EXPECT_TRUE(neighbor.point->cov.isApprox(expected_cov_SVD, 1e-5)) << "Wrong covariance (SVD)!";
      break;
    }
  }

  for (const auto& neighbor : neighbors_frob) {
    if ((neighbor.point->pos - Eigen::Vector3f(5.1, 2.1, 3.1)).norm() < 1e-5) {
      EXPECT_TRUE(neighbor.point->cov.isApprox(expected_cov_FROBENIUS, 1e-5)) << "Wrong covariance (FROBENIUS)!";
      break;
    }
  }

  for (const auto& neighbor : neighbors_min) {
    if ((neighbor.point->pos - Eigen::Vector3f(5.1, 2.1, 3.1)).norm() < 1e-5) {
      EXPECT_TRUE(neighbor.point->cov.isApprox(expected_cov_MIN_EIGENVALUE, 1e-5))
        << "Wrong covariance (MIN_EIGENVALUE)!";
      break;
    }
  }
}
