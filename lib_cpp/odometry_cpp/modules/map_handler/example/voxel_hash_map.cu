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

#include <iostream>
#include <tuple>
#include <vector>

#include "map_handler/voxel_hash_map.cuh"
#include "odometry_utils/utils.hpp"
#include "param_management_cpp/param_reference_manager.hpp"
#include "tsl_logger_cpp/reference_logger.hpp"
int main(int argc, char * argv[])
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
  auto map_ = tam::core::state::cuda::VoxelHashMap<tam::core::state::types::POINT_NORMAL>::from_config(pmg_.get(), logger_.get()); // NOLINT

  tam::pmg::MgmtInterface* pmg_raw = pmg_.get();
  pmg_raw->set_value("map.voxel_size", 1.0);
  pmg_raw->set_value("map.max_distance", 50.0);
  pmg_raw->set_value("map.frame_map", false);
  // Init threading after param overrides
  map_->init(1);

  // Load the pointcloud
  std::vector<tam::core::state::types::Point<tam::core::state::types::Point_XYZ>> frame =
    tam::core::state::utils::load_pointcloud_bin<tam::core::state::types::Point_XYZ>(
      argv[1]);  // NOLINT
  std::cout << "Loaded " << frame.size() << " points from " << argv[1] << std::endl;

  // create CUDA Stream
  cudaStream_t stream;
  cudaStreamCreate(&stream);

  // Add points to the map
  map_->add_points(frame, tam::core::state::types::POINT_NORMAL::MAX_POINTS_PER_VOXEL, 1, tam::core::state::types::POINT_NORMAL::NUM_NEIGHBORS, stream);  // NOLINT
  std::cout << "Points in the map: " << map_->num_points() << std::endl;

  tam::core::state::types::Point<tam::core::state::types::POINT_NORMAL> query;
  query.pos = Eigen::Vector3f(std::stod(argv[2]), std::stod(argv[3]), std::stod(argv[4]));
  std::cout << "Query point: " << query.pos.transpose() << std::endl;

  // synchronize the stream
  cudaStreamSynchronize(stream);
  cudaStreamDestroy(stream);

#ifdef USE_VISUALIZATION
  // Create rerun stream over TCP
  rerun::RecordingStream rec = tam::core::state::utils::spawn_rerun_stream("VoxelHashMap");

  // Get positions and normals
  thrust::device_vector<tam::core::state::types::Point<tam::core::state::types::POINT_NORMAL>>
    device_points = map_->get_cloud();
  std::vector<tam::core::state::types::Point<tam::core::state::types::POINT_NORMAL>> host_points(
    device_points.size());
  // Copy points from device to host
  thrust::copy(device_points.begin(), device_points.end(), host_points.begin());
  const std::tuple<
    std::vector<rerun::Position3D>, std::vector<rerun::Color>, std::vector<rerun::Vector3D>>
    rerun_points = tam::core::state::utils::points_normals2rerun(host_points, "Blue");

  rec.log(
    "points", rerun::Points3D(std::get<0>(rerun_points))
                .with_colors(std::get<1>(rerun_points))
                .with_radii({0.1f}));
  rec.log(
    "normals", rerun::Arrows3D::from_vectors(std::get<2>(rerun_points))
                 .with_origins(std::get<0>(rerun_points)));
#endif
  return 0;
}
