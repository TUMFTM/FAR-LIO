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
#include <gtest/gtest.h>

#include <algorithm>
#include <iostream>
#include <memory>
#include <random>
#include <string>
#include <vector>

#include "map_handler/voxel_hash_map.hpp"
#ifdef __CUDACC__
#include "map_handler/voxel_hash_map.cuh"
#endif
/**
 * @brief Initialize map with random points
 */
template <typename TConfig>
std::unique_ptr<tam::core::state::MapHandler<TConfig>> init_map(
  const size_t num_points = 10, const std::string& cov_reg = "SVD")
{
  // Construct from config and debug objects
  tam::core::state::types::MapConfig config;
  tam::core::state::types::MapDebug debug;
  config.voxel_size = 1.0;
  config.max_distance = 50.0;
  config.cov_regularization = cov_reg;  // Options: "SVD", "FROBENIUS", "MIN_EIGENVALUE"
#ifdef __CUDACC__
  auto map_ = tam::core::state::cuda::VoxelHashMap<TConfig>::from_config(config, debug);
#else
  auto map_ = tam::core::state::VoxelHashMap<TConfig>::from_config(config, debug);  // NOLINT
#endif
  map_->init();

  std::vector<tam::core::state::types::Point<tam::core::state::types::Point_XYZ>> frame{};
  if (num_points <= 10) {
    // Define a frame of points manually (for deterministic testing of neighbor search)
    tam::core::state::types::Point<tam::core::state::types::Point_XYZ> point_1, point_2, point_3, point_4, point_5,
      point_6, point_7, point_8, point_9, point_10;
    point_1.pos = Eigen::Vector3f(5.1, 2.1, 3.1);
    point_2.pos = Eigen::Vector3f(5.8, 2.5, 3.9);
    point_3.pos = Eigen::Vector3f(5.9, 2.9, 3.9);
    point_4.pos = Eigen::Vector3f(4.1, 1.0, 4.0);
    point_5.pos = Eigen::Vector3f(4.2, 1.1, 3.0);
    point_6.pos = Eigen::Vector3f(6.3, 3.2, 3.1);
    point_7.pos = Eigen::Vector3f(5.4, 2.3, 4.2);
    point_8.pos = Eigen::Vector3f(5.5, 2.4, 2.3);
    point_9.pos = Eigen::Vector3f(5.6, 2.5, 4.4);
    point_10.pos = Eigen::Vector3f(5.7, 2.6, 2.5);
    std::vector<tam::core::state::types::Point<tam::core::state::types::Point_XYZ>> frame_man{
      point_1, point_2, point_3, point_4, point_5, point_6, point_7, point_8, point_9, point_10};
    frame = frame_man;
  } else {
    // Generate random points within one voxel
    frame.reserve(num_points);
    std::random_device rd;
    std::mt19937 gen(rd());
    std::uniform_real_distribution<> dis(-config.voxel_size / 2, config.voxel_size / 2);

    for (size_t i = 0; i < num_points; ++i) {
      tam::core::state::types::Point<tam::core::state::types::Point_XYZ> point;
      point.pos = Eigen::Vector3f(0.5f + dis(gen), 0.5f + dis(gen), 0.5f + dis(gen));
      frame.push_back(point);
    }
  }
  // Add points to the map
  map_->add_points(frame);
  return map_;
}
