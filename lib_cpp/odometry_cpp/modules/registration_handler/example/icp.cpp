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

#include "registration_handler/icp.hpp"

#include <iostream>
#include <sophus/se3.hpp>
#include <tuple>
#include <vector>

#include "map_handler/voxel_hash_map.hpp"
#include "odometry_utils/utils.hpp"
#include "param_management_cpp/param_reference_manager.hpp"
#include "tsl_logger_cpp/reference_logger.hpp"
#include "voxel_tools/voxel_tools.hpp"

int main(int argc, char* argv[])
{
  // Check the number of arguments
  if (argc < 4) {
    std::cerr << "Usage: " << argv[0] << " <map_path> <frame_path> <initial_guess_path>" << std::endl;
    return 1;
  }
  for (int i = 0; i < argc; ++i) {
    std::ifstream file(argv[1]);
    if (!file) {
      std::cerr << "Error: Cannot open file " << argv[1] << std::endl;
      return 1;
    }
  }

  // Using default parameter
  // Initialize ICP
  // clang-format off
  // Construct from param manager and logger
  tam::pmg::ParamReferenceManager::UniquePtr pmg_ =  std::make_unique<tam::pmg::ParamReferenceManager>(); // NOLINT
  tam::tsl::ReferenceLogger::UniquePtr logger_ = std::make_unique<tam::tsl::ReferenceLogger>(); // NOLINT
  auto registration_ = tam::core::state::ICP<tam::core::state::types::ICP_EXT>::from_config(pmg_.get(), logger_.get()); // NOLINT
  // clang-format on

  // init multi-threading and set number of threads
  tam::pmg::MgmtInterface* pmg_raw = pmg_.get();
  pmg_raw->set_value("registration.solver_type", "GaussNewton");
  pmg_raw->set_value("registration.max_iter", std::int64_t{1000});
  pmg_raw->set_value("registration.max_inner_iter", std::int64_t{20});
  pmg_raw->set_value("registration.max_time", 5000.0);
  pmg_raw->set_value("registration.convergence_criterion", 5e-3);
  pmg_raw->set_value("registration.damping_factor", 0.0);
  pmg_raw->set_value("registration.damping_scale", 2.0);
  registration_->init(3);

  // Initialize a map
  // clang-format off
  // Construct from param manager and logger
  auto map_ = tam::core::state::VoxelHashMap<tam::core::state::types::ICP_EXT>::from_config(pmg_.get(), logger_.get()); // NOLINT
  map_->init();

  // Load the map
  std::vector<tam::core::state::types::Point<tam::core::state::types::Point_XYZ>> map = tam::core::state::utils::load_pointcloud_bin<tam::core::state::types::Point_XYZ>(argv[1]); // NOLINT
  std::cout << "Loaded " << map.size() << " points from " << argv[1] << std::endl;
  // Add points to the map
  map_->add_points(map);

  std::cout << "Voxels in the map: " << map_->get_debug().num_voxel << std::endl;
  std::cout << "Points in the map: " << map_->get_debug().num_points << std::endl;

  std::vector<tam::core::state::types::Point<tam::core::state::types::ICP_EXT>> frame = tam::core::state::utils::load_pointcloud_bin<tam::core::state::types::ICP_EXT>(argv[2]); // NOLINT
  std::cout << "Loaded " << frame.size() << " points from " << argv[2] << std::endl;
  // Load the initial guess
  Sophus::SE3f init_guess = tam::core::state::utils::load_pose<tam::core::state::types::ICP_EXT>(argv[3]); // NOLINT

  // Downsample the frame
  const auto & [frame_registration, frame_map] = tam::core::state::voxel_doubledownsample(frame, map_->get_config().voxel_size);  // NOLINT

  const float sigma = 6.0;
  const Sophus::SE3f T_icp = registration_->register_frame(frame_registration, map_.get(), init_guess, 3.0 * sigma, sigma);  // NOLINT
  map_->update_points(frame_map, T_icp);
  // clang-format on

  std::cout << "Voxels in the map: " << map_->get_debug().num_voxel << std::endl;
  std::cout << "Points in the map: " << map_->get_debug().num_points << std::endl;
  std::cout << "Voxels added: " << map_->get_debug().voxel_added << std::endl;
  std::cout << "Points added: " << map_->get_debug().points_added << std::endl;
  std::cout << "Voxels removed: " << map_->get_debug().voxel_removed << std::endl;
  std::cout << "Points removed: " << map_->get_debug().points_removed << std::endl;
  std::cout << "Points in the frame: " << registration_->get_debug().num_points_frame << std::endl;
  std::cout << "Initial guess: " << std::endl << init_guess.matrix() << std::endl;
  std::cout << "Transformation from frame to map: " << std::endl << T_icp.matrix() << std::endl;
  std::cout << "Converged: " << registration_->get_registration_status().converged << std::endl;
  std::cout << "Registration time: " << registration_->get_debug().registration_time << " ms" << std::endl;
  std::cout << "Damping Factor: " << registration_->get_debug().damping_factor << std::endl;
  std::cout << "Iterations: " << registration_->get_debug().num_iter << std::endl;
  if (registration_->get_config().solver_type == "LevenbergMarquardt") {
    std::cout << "Error: " << std::get<double>(registration_->get_debug().conditional["error"]) << std::endl;
    std::cout << "Inner Iterations: "
              << std::get<std::int64_t>(registration_->get_debug().conditional["num_inner_iter"]) << std::endl;
  }
#ifdef USE_VISUALIZATION
  // Create rerun stream over TCP
  rerun::RecordingStream rec = tam::core::state::utils::spawn_rerun_stream("ICP");

  // Convert map to rerun format
  const std::tuple<std::vector<rerun::Position3D>, std::vector<rerun::Color>> rerun_map =
    tam::core::state::utils::points2rerun(map_->get_cloud(), "Blue");

  // Convert frame to rerun format with init guess and final transformation
  const std::tuple<std::vector<rerun::Position3D>, std::vector<rerun::Color>> rerun_frame_init =
    tam::core::state::utils::points2rerun(frame, "Orange", init_guess);
  const std::tuple<std::vector<rerun::Position3D>, std::vector<rerun::Color>> rerun_frame_final =
    tam::core::state::utils::points2rerun(frame, "Green", T_icp);

  rec.log("map", rerun::Points3D(std::get<0>(rerun_map)).with_colors(std::get<1>(rerun_map)).with_radii({0.3f}));
  rec.log("frame_init",
    rerun::Points3D(std::get<0>(rerun_frame_init)).with_colors(std::get<1>(rerun_frame_init)).with_radii({0.3f}));
  rec.log("frame_final",
    rerun::Points3D(std::get<0>(rerun_frame_final)).with_colors(std::get<1>(rerun_frame_init)).with_radii({0.3f}));
#endif
  return 0;
}
