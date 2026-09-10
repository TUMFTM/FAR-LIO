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

#include <cuda_runtime.h>
#include <gtest/gtest.h>

#include <algorithm>
#include <iostream>
#include <memory>
#include <random>
#include <string>
#include <vector>

#include "test_utils.hpp"

/**
 * @brief Test construction of VoxelHashMapCuda from param manager and logger
 */
TEST(VoxelHashMapCuda, BuildPmgLogger)
{
  // Check if CUDA device is available
  CHECK_CUDA_ERROR();
  // Construct from param manager and logger
  tam::pmg::ParamReferenceManager::UniquePtr pmg_ = std::make_unique<tam::pmg::ParamReferenceManager>();
  tam::tsl::ReferenceLogger::UniquePtr logger_ = std::make_unique<tam::tsl::ReferenceLogger>();
  auto map_ =
    tam::core::state::cuda::VoxelHashMap<tam::core::state::types::POINT_NORMAL>::from_config(pmg_.get(), logger_.get());

  tam::pmg::MgmtInterface* pmg_raw = pmg_.get();
  pmg_raw->set_value("map.voxel_size", 5.0);

  EXPECT_EQ(map_->get_config().voxel_size, 5.0) << "Failed to construct VoxelHashMapCuda from param manager and logger";
}

/**
 * @brief Check correct number of points after map initialization
 */
TEST(VoxelHashMapCuda, NumPoints)
{
  // Check if CUDA device is available
  CHECK_CUDA_ERROR();

  auto map_ = init_map<tam::core::state::types::POINT_NORMAL>();
  EXPECT_EQ(map_->num_points(), 10) << "Wrong amount of points in map";
}

/**
 * @brief Check correct map size after adding points
 */
TEST(VoxelHashMapCuda, AddPoints)
{
  // Check if CUDA device is available
  CHECK_CUDA_ERROR();
  // Generate random points in one voxel
  auto map_ = init_map<tam::core::state::types::POINT_NORMAL>(100);
  EXPECT_EQ(map_->num_points(), tam::core::state::types::POINT_NORMAL::MAX_POINTS_PER_VOXEL)
    << "Wrong amount of points per voxel";
}

/**
 * @brief Check correct correspondence calculation on the GPU
 */
TEST(VoxelHashMapCuda, SearchNeighbor)
{
  // Check if CUDA device is available
  CHECK_CUDA_ERROR();
  auto map_ = init_map<tam::core::state::types::POINT_NORMAL>();

  // Create query point
  tam::core::state::types::Point<tam::core::state::types::POINT_NORMAL> query;
  query.pos = Eigen::Vector3f(5.05, 2.3, 3.15);
  thrust::device_vector<tam::core::state::types::Point<tam::core::state::types::POINT_NORMAL>> query_vec;
  query_vec.push_back(query);

  // Init correspondences vector
  thrust::device_vector<tam::core::state::types::Correspondence<tam::core::state::types::POINT_NORMAL>> correspondences(
    1);

  // Search for correspondence
  map_->search_closest_neighbor(query_vec, correspondences);

  // Copy correspondence to host
  std::vector<tam::core::state::types::Correspondence<tam::core::state::types::POINT_NORMAL>> correspondences_host(
    correspondences.size());
  thrust::copy(correspondences.begin(), correspondences.end(), correspondences_host.begin());

  // Compare results (assuming we get back host-accessible data)
  EXPECT_NEAR(correspondences_host.front().map.pos.x(), 5.1f, 1e-5) << "Wrong closest neighbor found (x coordinate)!";
  EXPECT_NEAR(correspondences_host.front().map.pos.y(), 2.1f, 1e-5) << "Wrong closest neighbor found (y coordinate)!";
  EXPECT_NEAR(correspondences_host.front().map.pos.z(), 3.1f, 1e-5) << "Wrong closest neighbor found (z coordinate)!";
}

/**
 * @brief Check correct neighbors search on the GPU
 */
TEST(VoxelHashMapCuda, SearchNeighbors)
{
  // Check if CUDA device is available
  CHECK_CUDA_ERROR();
  auto map_ = init_map<tam::core::state::types::POINT_NORMAL>();
  std::vector<float> point_1_distances{0.000, 1.136, 1.386, 1.738, 1.349, 1.628, 1.158, 0.943, 1.449, 0.985};

  // Capture neighbors
  auto neighbors = map_->get_neighbors();

  // Copy neighbors to host
  std::vector<tam::core::state::types::Neighbors<tam::core::state::types::POINT_NORMAL>> neighbors_host(
    neighbors.size());
  thrust::copy(neighbors.begin(), neighbors.end(), neighbors_host.begin());
  // Copy the actual points to host, as the neighbors only contain device pointers
  std::vector<tam::core::state::types::Point<tam::core::state::types::POINT_NORMAL>> points_host(neighbors_host.size());
  for (size_t i = 0; i < neighbors_host.size(); ++i) {
    cudaMemcpy(&points_host[i], neighbors_host[i].point,
      sizeof(tam::core::state::types::Point<tam::core::state::types::POINT_NORMAL>), cudaMemcpyDeviceToHost);
  }

  EXPECT_EQ(neighbors_host.size(), 10) << "Wrong amount of neighbors found in calculation!";
  EXPECT_EQ(neighbors_host.front().num_neighbors, tam::core::state::types::POINT_NORMAL::NUM_NEIGHBORS)
    << "Wrong amount of neighbors found!";
  // Check neighbors exemplary for first point
  int i = 0;
  for (const auto& neighbor : neighbors_host) {
    if ((points_host[i].pos - Eigen::Vector3f(5.1, 2.1, 3.1)).norm() < 1e-5) {
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
    i++;
  }
}

/**
 * @brief Check correct normal computation on the GPU
 */
TEST(VoxelHashMapCuda, Normals)
{
  // Check if CUDA device is available
  CHECK_CUDA_ERROR();

  auto map_ = init_map<tam::core::state::types::POINT_NORMAL>();

  // Capture neighbors
  auto neighbors = map_->get_neighbors();

  // Copy neighbors to host
  std::vector<tam::core::state::types::Neighbors<tam::core::state::types::POINT_NORMAL>> neighbors_host(
    neighbors.size());
  thrust::copy(neighbors.begin(), neighbors.end(), neighbors_host.begin());
  // Copy the actual points to host, as the neighbors only contain device pointers
  std::vector<tam::core::state::types::Point<tam::core::state::types::POINT_NORMAL>> points_host(neighbors_host.size());
  for (size_t i = 0; i < neighbors_host.size(); ++i) {
    cudaMemcpy(&points_host[i], neighbors_host[i].point,
      sizeof(tam::core::state::types::Point<tam::core::state::types::POINT_NORMAL>), cudaMemcpyDeviceToHost);
  }

  EXPECT_EQ(neighbors_host.size(), 10) << "Wrong amount of neighbors found in calculation!";
  EXPECT_EQ(neighbors_host.front().num_neighbors, tam::core::state::types::POINT_NORMAL::NUM_NEIGHBORS)
    << "Wrong amount of neighbors found!";
  int i = 0;
  // Check normal exemplary for first point
  for (const auto& neighbor : neighbors) {
    if ((points_host[i].pos - Eigen::Vector3f(5.1, 2.1, 3.1)).norm() < 1e-5) {
      // Check normals for computed neighbor
      // Compare results (assuming we get back host-accessible data)
      EXPECT_NEAR(points_host[i].normal.x(), -0.561368f, 1e-5) << "Wrong normal (x coordinate)!";
      EXPECT_NEAR(points_host[i].normal.y(), 0.826427f, 1e-5) << "Wrong normal (y coordinate)!";
      EXPECT_NEAR(points_host[i].normal.z(), 0.0434006f, 1e-5) << "Wrong normal (z coordinate)!";
      break;
    }
  }
}

/**
 * @brief Check correct covariance computation on the GPU
 */
TEST(VoxelHashMapCuda, Covariances)
{
  // Check if CUDA device is available
  CHECK_CUDA_ERROR();

  // Test SVD regularization
  auto map_svd = init_map<tam::core::state::types::POINT_NORMAL>(10, "SVD");
  auto neighbors_svd = map_svd->get_neighbors();
  // Copy neighbors to host
  std::vector<tam::core::state::types::Neighbors<tam::core::state::types::POINT_NORMAL>> neighbors_svd_host(
    neighbors_svd.size());
  thrust::copy(neighbors_svd.begin(), neighbors_svd.end(), neighbors_svd_host.begin());
  // Copy the actual points to host, as the neighbors only contain device pointers
  std::vector<tam::core::state::types::Point<tam::core::state::types::POINT_NORMAL>> points_svd_host(
    neighbors_svd_host.size());
  for (size_t i = 0; i < neighbors_svd_host.size(); ++i) {
    cudaMemcpy(&points_svd_host[i], neighbors_svd_host[i].point,
      sizeof(tam::core::state::types::Point<tam::core::state::types::POINT_NORMAL>), cudaMemcpyDeviceToHost);
  }

  // Test FROBENIUS regularization
  auto map_frob = init_map<tam::core::state::types::POINT_NORMAL>(10, "FROBENIUS");
  auto neighbors_frob = map_frob->get_neighbors();
  // Copy neighbors to host
  std::vector<tam::core::state::types::Neighbors<tam::core::state::types::POINT_NORMAL>> neighbors_frob_host(
    neighbors_frob.size());
  thrust::copy(neighbors_frob.begin(), neighbors_frob.end(), neighbors_frob_host.begin());
  // Copy the actual points to host, as the neighbors only contain device pointers
  std::vector<tam::core::state::types::Point<tam::core::state::types::POINT_NORMAL>> points_frob_host(
    neighbors_frob_host.size());
  for (size_t i = 0; i < neighbors_frob_host.size(); ++i) {
    cudaMemcpy(&points_frob_host[i], neighbors_frob_host[i].point,
      sizeof(tam::core::state::types::Point<tam::core::state::types::POINT_NORMAL>), cudaMemcpyDeviceToHost);
  }

  // Test MIN_EIGENVALUE regularization
  auto map_min = init_map<tam::core::state::types::POINT_NORMAL>(10, "MIN_EIGENVALUE");
  auto neighbors_min = map_min->get_neighbors();
  // Copy neighbors to host
  std::vector<tam::core::state::types::Neighbors<tam::core::state::types::POINT_NORMAL>> neighbors_min_host(
    neighbors_min.size());
  thrust::copy(neighbors_min.begin(), neighbors_min.end(), neighbors_min_host.begin());
  // Copy the actual points to host, as the neighbors only contain device pointers
  std::vector<tam::core::state::types::Point<tam::core::state::types::POINT_NORMAL>> points_min_host(
    neighbors_min_host.size());
  for (size_t i = 0; i < neighbors_min_host.size(); ++i) {
    cudaMemcpy(&points_min_host[i], neighbors_min_host[i].point,
      sizeof(tam::core::state::types::Point<tam::core::state::types::POINT_NORMAL>), cudaMemcpyDeviceToHost);
  }

  Eigen::Matrix3f expected_cov_SVD, expected_cov_FROBENIUS, expected_cov_MIN_EIGENVALUE;
  expected_cov_SVD << 0.685181, 0.463466, 0.0243394, 0.463466, 0.317701, -0.0358316, 0.0243394, -0.0358316, 0.998118;
  expected_cov_FROBENIUS << 31.1507, 20.4267, 1.02138, 20.4267, 15.6264, -14.2987, 1.02138, -14.2987, 286.484;
  expected_cov_MIN_EIGENVALUE << 0.060013, 0.0399809, 0.00199909, 0.0399809, 0.0296282, -0.0279985, 0.00199909,
    -0.0279985, 0.560000;

  // Check for each regularization type
  int i = 0;
  for (const auto& neighbor : neighbors_svd_host) {
    if ((points_svd_host[i].pos - Eigen::Vector3f(5.1, 2.1, 3.1)).norm() < 1e-5) {
      EXPECT_TRUE(points_svd_host[i].cov.isApprox(expected_cov_SVD, 1e-5)) << "Wrong covariance (SVD)!";
      break;
    }
    i++;
  }

  i = 0;
  for (const auto& neighbor : neighbors_frob_host) {
    if ((points_frob_host[i].pos - Eigen::Vector3f(5.1, 2.1, 3.1)).norm() < 1e-5) {
      EXPECT_TRUE(points_frob_host[i].cov.isApprox(expected_cov_FROBENIUS, 1e-3)) << "Wrong covariance (FROBENIUS)!";
      break;
    }
    i++;
  }

  i = 0;
  for (const auto& neighbor : neighbors_min_host) {
    if ((points_min_host[i].pos - Eigen::Vector3f(5.1, 2.1, 3.1)).norm() < 1e-5) {
      EXPECT_TRUE(points_min_host[i].cov.isApprox(expected_cov_MIN_EIGENVALUE, 1e-5))
        << "Wrong covariance (MIN_EIGENVALUE)!";
      break;
    }
    i++;
  }
}
