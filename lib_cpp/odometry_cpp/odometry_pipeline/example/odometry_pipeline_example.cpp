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

#include "odometry_pipeline/odometry_pipeline.hpp"
#include "param_management_cpp/param_reference_manager.hpp"
#include "tsl_logger_cpp/reference_logger.hpp"

int main()
{
  // clang-format off
  // Construct from param manager and logger
  tam::pmg::ParamReferenceManager::UniquePtr pmg_ =  std::make_unique<tam::pmg::ParamReferenceManager>(); // NOLINT
  tam::tsl::ReferenceLogger::UniquePtr logger_ = std::make_unique<tam::tsl::ReferenceLogger>(); // NOLINT

// Note: #ifdef __CUDACC__ not working in this context, as it's a cpp file -> always use CPU version
#ifdef __CUDACC__
  auto odom_pipeline = tam::core::state::OdometryPipeline<tam::core::state::types::CUDA_ICP_EXT>::from_config(pmg_.get(), logger_.get()); // NOLINT
#else
  auto odom_pipeline = tam::core::state::OdometryPipeline<tam::core::state::types::ICP_EXT>::from_config(pmg_.get(), logger_.get()); // NOLINT
#endif
  // clang-format on

  tam::pmg::MgmtInterface* pmg_raw = pmg_.get();
  // Diagnostic parameters
  pmg_raw->set_value("diagnostic.frame_outdated", 100.0);      // NOLINT
  pmg_raw->set_value("diagnostic.check_forward", true);        // NOLINT
  pmg_raw->set_value("diagnostic.min_motion_threshold", 5.0);  // NOLINT
  pmg_raw->set_value("diagnostic.check_ellipsis", true);       // NOLINT
  pmg_raw->set_value("diagnostic.ellipsis_size_s", 5.0);       // NOLINT
  pmg_raw->set_value("diagnostic.ellipsis_size_d", 3.0);       // NOLINT
  pmg_raw->set_value("diagnostic.check_time_diff", true);      // NOLINT
  pmg_raw->set_value("diagnostic.max_time_diff", 0.05);        // NOLINT

  // Map parameters
  pmg_raw->set_value("map.voxel_size", 1.0);     // NOLINT
  pmg_raw->set_value("map.max_distance", 50.0);  // NOLINT

  // Model parameters
  pmg_raw->set_value("model.initial_pos_x", 1.0);  // NOLINT
  pmg_raw->set_value("model.initial_pos_y", 3.0);  // NOLINT
  pmg_raw->set_value("model.initial_pos_z", 2.0);  // NOLINT
  pmg_raw->set_value("model.initial_rot_x", 1.0);  // NOLINT
  pmg_raw->set_value("model.initial_rot_y", 2.0);  // NOLINT
  pmg_raw->set_value("model.initial_rot_z", 3.0);  // NOLINT
  pmg_raw->set_value("model.initial_rot_w", 4.0);  // NOLINT

  // Registration parameters
  pmg_raw->set_value("registration.max_iter", std::int64_t{500});   // NOLINT
  pmg_raw->set_value("registration.max_time", 150.0);               // NOLINT
  pmg_raw->set_value("registration.convergence_criterion", 1e-6);   // NOLINT
  pmg_raw->set_value("registration.num_threads", std::int64_t{1});  // NOLINT

  // Threshold parameters
  pmg_raw->set_value("threshold.initial_threshold", 6.0);          // NOLINT
  pmg_raw->set_value("threshold.min_motion_threshold", 5.0);       // NOLINT
  pmg_raw->set_value("threshold.max_correspondence_range", 20.0);  // NOLINT

  std::cout << "Odometry pipeline created" << std::endl;
  std::cout << "Max registration time: " << odom_pipeline->get_registration_config().max_time << std::endl;
  return 0;
}
