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

#include "map_handler/voxel_hash_map.hpp"

#include <chrono>
#include <iostream>
#include <memory>
#include <tuple>
#include <vector>

#include "odometry_utils/utils.hpp"
#include "param_management_cpp/param_reference_manager.hpp"
#include "tsl_logger_cpp/reference_logger.hpp"

int main(int argc, char* argv[])
{
  // Check the number of arguments
  if (argc < 5) {
    std::cerr << "Usage: " << argv[0] << "<pointcloud_path> <x> <y> <z>" << std::endl
              << "(Example query point using map.bin: 86.2 -442.6 34.5)" << std::endl;
    return 1;
  }
  for (int i = 0; i < argc; ++i) {
    std::ifstream file(argv[1]);
    if (!file) {
      std::cerr << "Error: Cannot open file " << argv[1] << std::endl;
      return 1;
    }
  }

  // clang-format off
  // Construct from param manager and logger
  tam::pmg::ParamReferenceManager::UniquePtr pmg_ =  std::make_unique<tam::pmg::ParamReferenceManager>(); // NOLINT
  tam::tsl::ReferenceLogger::UniquePtr logger_ = std::make_unique<tam::tsl::ReferenceLogger>(); // NOLINT
  std::unique_ptr<tam::core::state::MapHandler<tam::core::state::types::POINT_NORMAL>> map_ =
    tam::core::state::VoxelHashMap<tam::core::state::types::POINT_NORMAL>::from_config(pmg_.get(), logger_.get()); // NOLINT

  tam::pmg::MgmtInterface* pmg_raw = pmg_.get();
  pmg_raw->set_value("map.voxel_size", 1.0);
  pmg_raw->set_value("map.max_distance", 50.0);
  pmg_raw->set_value("map.frame_map", false);
  pmg_raw->set_value("map.cov_regularization", "SVD");  // Options: "SVD", "FROBENIUS", "MIN_EIGENVALUE"  // NOLINT
  // Init threading after param overrides
  map_->init(3);
  // clang-format on

  // Load the pointcloud
  std::vector<tam::core::state::types::Point<tam::core::state::types::Point_XYZ>> frame =
    tam::core::state::utils::load_pointcloud_bin<tam::core::state::types::Point_XYZ>(argv[1]);  // NOLINT
  std::cout << "Loaded " << frame.size() << " points from " << argv[1] << std::endl;
  // Add points to the map
  map_->add_points(frame);
  std::cout << "Points in the map: " << map_->num_points() << std::endl;

  tam::core::state::types::Point<tam::core::state::types::POINT_NORMAL> query;
  query.pos = Eigen::Vector3f(std::stod(argv[2]), std::stod(argv[3]), std::stod(argv[4]));
  std::cout << "Query point: " << query.pos.transpose() << std::endl;

  // Search for the closest neighbor
  auto start = std::chrono::high_resolution_clock::now();
  const auto& correspondence = map_->search_closest_neighbor(query, 1);
  auto time = (std::chrono::high_resolution_clock::now() - start).count();
  std::cout << "Closest neighbor: " << correspondence.map.pos.transpose() << " at distance: " << correspondence.distance
            << std::endl
            << "Time: " << time << " ns" << std::endl;

#ifdef USE_VISUALIZATION
  // Create rerun stream over TCP
  rerun::RecordingStream rec = tam::core::state::utils::spawn_rerun_stream("VoxelHashMap");

  // Get positions and normals
  const std::tuple<std::vector<rerun::Position3D>, std::vector<rerun::Color>, std::vector<rerun::Vector3D>>
    rerun_points = tam::core::state::utils::points_normals2rerun(map_->get_cloud(), "Blue");

  rec.log(
    "points", rerun::Points3D(std::get<0>(rerun_points)).with_colors(std::get<1>(rerun_points)).with_radii({0.1f}));
  rec.log("normals", rerun::Arrows3D::from_vectors(std::get<2>(rerun_points)).with_origins(std::get<0>(rerun_points)));
#endif
  return 0;
}
