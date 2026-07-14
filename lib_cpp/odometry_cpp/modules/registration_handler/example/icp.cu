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
#include <sophus/se3.hpp>

#include "map_handler/voxel_hash_map.cuh"
#include "odometry_utils/utils.hpp"
#include "param_management_cpp/param_reference_manager.hpp"
#include "registration_handler/icp.cuh"
#include "tsl_logger_cpp/reference_logger.hpp"
#include "voxel_tools/voxel_tools.cuh"
int main(int argc, char * argv[])
{
  // Check the number of arguments
  if (argc < 4) {
    std::cerr << "Usage: " << argv[0] << " <map_path> <frame_path> <initial_guess_path>"
              << std::endl;
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
  auto map_ = tam::core::state::cuda::VoxelHashMap<tam::core::state::types::CUDA_ICP_EXT>::from_config(pmg_.get(), logger_.get()); // NOLINT
  auto registration_ = tam::core::state::cuda::ICP<tam::core::state::types::CUDA_ICP_EXT>::from_config(pmg_.get(), logger_.get()); // NOLINT
  // clang-format on

  // init cuda and set parameter
  tam::pmg::MgmtInterface * pmg_raw = pmg_.get();
  pmg_raw->set_value("map.max_distance", 1000.0);
  pmg_raw->set_value("registration.solver_type", "GaussNewton");
  pmg_raw->set_value("registration.max_iter", std::int64_t{1000});
  pmg_raw->set_value("registration.max_inner_iter", std::int64_t{20});
  pmg_raw->set_value("registration.max_time", 5000.0);
  pmg_raw->set_value("registration.convergence_criterion", 5e-3);
  pmg_raw->set_value("registration.damping_factor", 0.0);
  pmg_raw->set_value("registration.damping_scale", 10.0);
  map_->init(1);
  registration_->init(1);

  // create CUDA Stream
  cudaStream_t stream;
  cudaStreamCreate(&stream);

  // Load the map
  std::vector<tam::core::state::types::Point<tam::core::state::types::Point_XYZ>> map = tam::core::state::utils::load_pointcloud_bin<tam::core::state::types::Point_XYZ>(argv[1]); // NOLINT
  std::cout << "Loaded " << map.size() << " points from " << argv[1] << std::endl;
  // Add points to the map
  map_->add_points(map, tam::core::state::types::POINT_NORMAL::MAX_POINTS_PER_VOXEL, 1, tam::core::state::types::CUDA_ICP_EXT::NUM_NEIGHBORS, stream);  // NOLINT

  size_t free_mem, total_mem;
  cudaError_t err = cudaMemGetInfo(&free_mem, &total_mem);

  if (err != cudaSuccess) {
      std::cerr << "cudaMemGetInfo failed: " << cudaGetErrorString(err) << std::endl;
      return 1;
  }

  size_t used_mem = total_mem - free_mem;
  std::cout << "Voxels in the map: " << map_->get_debug().num_voxel << std::endl;
  std::cout << "Points in the map: " << map_->get_debug().num_points << std::endl;
  std::cout << "CUDA Memory usage after map loading:" << std::endl;
  std::cout << "Total memory: " << total_mem / (1024 * 1024) << " MB" << std::endl;
  std::cout << "Free memory: " << free_mem / (1024 * 1024) << " MB" << std::endl;
  std::cout << "Used memory: " << used_mem / (1024 * 1024) << " MB" << std::endl;

  std::vector<tam::core::state::types::Point<tam::core::state::types::CUDA_ICP_EXT>> frame = tam::core::state::utils::load_pointcloud_bin<tam::core::state::types::CUDA_ICP_EXT>(argv[2]); // NOLINT
  std::cout << "Loaded " << frame.size() << " points from " << argv[2] << std::endl;
  // Load the initial guess
  Sophus::SE3f init_guess = tam::core::state::utils::load_pose<tam::core::state::types::CUDA_ICP_EXT>(argv[3]);  // NOLINT

  const auto & [frame_registration, frame_map] = tam::core::state::cuda::voxel_doubledownsample<tam::core::state::types::CUDA_ICP_EXT>(frame, map_->get_config().voxel_size);  // NOLINT
  const float sigma = 6.0;
  const Sophus::SE3f T_icp = registration_->register_frame(frame_registration, map_.get(), init_guess, 3.0 * sigma, sigma, stream);  // NOLINT
  map_->update_points(frame_map, T_icp, tam::core::state::types::POINT_NORMAL::MAX_POINTS_PER_VOXEL, 1, tam::core::state::types::CUDA_ICP_EXT::NUM_NEIGHBORS, stream);  // NOLINT

  // clang-format on

  // synchronize the stream
  cudaStreamSynchronize(stream);
  cudaStreamDestroy(stream);

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
  std::cout << "Registration time: " << registration_->get_debug().registration_time << " ms"
            << std::endl;
  std::cout << "Damping Factor: " << registration_->get_debug().damping_factor << std::endl;
  std::cout << "Iterations: " << registration_->get_debug().num_iter << std::endl;
if (registration_->get_config().solver_type == "LevenbergMarquardt") {
  std::cout << "Error: " << std::get<double>(registration_->get_debug().conditional["error"])
            << std::endl;
  std::cout << "Inner Iterations: "
            << std::get<std::int64_t>(registration_->get_debug().conditional["num_inner_iter"])
            << std::endl;
}
  return 0;
}
