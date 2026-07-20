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

#include <iostream>
#include <string>
#include <vector>

#ifdef __CUDACC__
#include "map_handler/voxel_hash_map.cuh"
#include "registration_handler/gicp.cuh"
#include "registration_handler/icp.cuh"
#endif

#include "map_handler/voxel_hash_map.hpp"
#include "param_management_cpp/param_reference_manager.hpp"
#include "registration_handler/gicp.hpp"
#include "registration_handler/icp.hpp"
#include "tsl_logger_cpp/reference_logger.hpp"

/**
 * @brief Initialize points for testing
 */
template <typename TConfig>
std::vector<tam::core::state::types::Point<TConfig>> generate_points(const Sophus::SE3f& T = Sophus::SE3f())
{
  // Define a frame of points manually (for deterministic testing of neighbor search)
  tam::core::state::types::Point<TConfig> point_1, point_2, point_3, point_4, point_5, point_6, point_7, point_8,
    point_9, point_10;
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
  std::vector<tam::core::state::types::Point<TConfig>> frame{
    point_1, point_2, point_3, point_4, point_5, point_6, point_7, point_8, point_9, point_10};
  // Transform points in-place
  tam::core::state::utils::transform_points(T, frame);
  return frame;
}

/**
 * @brief Parameter struct for GICP registration tests
 */
struct TestParams {
  std::string cov_regularization;
  std::string solver_type;

  // For readable test names
  std::string string() const { return cov_regularization + "_" + solver_type; }
};
