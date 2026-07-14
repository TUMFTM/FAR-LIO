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
// NOTE: If you include this header, you MUST instantiate the defined parameterized TestSuite
#pragma once

#include <iostream>
#include <string>
#include <vector>

#include "test_utils.hpp"
/**
 * @brief Parameterized test class for ICP registration
 */
class ICPParameterizedTest : public ::testing::TestWithParam<TestParams>
{
protected:
  void SetUp() override {}
  void TearDown() override {}
};
/**
 * @brief Test registration of a frame to the map
 */
TEST_P(ICPParameterizedTest, ICPRegisterFrameParameterized)
{
  const auto & params = GetParam();
  // Initialize ICP
  // clang-format off
  // Construct from param manager and logger
  tam::pmg::ParamReferenceManager::UniquePtr pmg_ =  std::make_unique<tam::pmg::ParamReferenceManager>(); // NOLINT
  tam::tsl::ReferenceLogger::UniquePtr logger_ = std::make_unique<tam::tsl::ReferenceLogger>(); // NOLINT
  #ifdef __CUDACC__
  // Check if CUDA device is available
  CHECK_CUDA_ERROR();
  auto map_ = tam::core::state::cuda::VoxelHashMap<tam::core::state::types::CUDA_ICP_EXT>::from_config(pmg_.get(), logger_.get()); // NOLINT
  auto registration_ = tam::core::state::cuda::ICP<tam::core::state::types::CUDA_ICP_EXT>::from_config(pmg_.get(), logger_.get()); // NOLINT
  #else
  auto map_ = tam::core::state::VoxelHashMap<tam::core::state::types::ICP_EXT>::from_config(pmg_.get(), logger_.get()); // NOLINT
  auto registration_ =  tam::core::state::ICP<tam::core::state::types::ICP_EXT>::from_config(pmg_.get(), logger_.get()); // NOLINT
  #endif
  // clang-format on
  tam::pmg::MgmtInterface * pmg_raw = pmg_.get();
  pmg_raw->set_value("map.frame_map", false);
  pmg_raw->set_value("map.voxel_size", 1.0);
  pmg_raw->set_value("map.max_distance", 100.0);
  pmg_raw->set_value("map.cov_regularization", params.cov_regularization);  // not used here
  pmg_raw->set_value("registration.solver_type", params.solver_type);
  pmg_raw->set_value("registration.damping_factor", 0.0);
  pmg_raw->set_value("registration.convergence_criterion", 5.0e-3);
  map_->init();
  registration_->init();

  // Define a frame of points
  std::vector<tam::core::state::types::Point<tam::core::state::types::Point_XYZ>> frame =
    generate_points<tam::core::state::types::Point_XYZ>(Sophus::SE3f());

  map_->add_points(frame);
  // clang-format on

  Sophus::SE3f trans = Sophus::SE3f(
    Sophus::SE3f::QuaternionType(0.50, 0.004, 0.02, 0.86), Sophus::SE3f::Point(3.0, 2.0, 5.0));

  // Transform points
#ifdef __CUDACC__
  std::vector<tam::core::state::types::Point<tam::core::state::types::CUDA_ICP_EXT>>
    transformed_frame_h = generate_points<tam::core::state::types::CUDA_ICP_EXT>(trans);
  thrust::device_vector<tam::core::state::types::Point<tam::core::state::types::CUDA_ICP_EXT>>
    transformed_frame(transformed_frame_h.begin(), transformed_frame_h.end());
#else
  std::vector<tam::core::state::types::Point<tam::core::state::types::ICP_EXT>> transformed_frame =
    generate_points<tam::core::state::types::ICP_EXT>(trans);
#endif

  // Generate initial guess
  Sophus::SE3f init_guess = trans.inverse();
  std::default_random_engine generator;
  std::normal_distribution<float> quaternion_distribution_init(0.05, 0.001);  // Mean 0, Stddev 0.01
  std::normal_distribution<float> translation_distribution_init(1.0, 0.1);  // Mean 0.1, Stddev 0.1
  Sophus::SE3f init_guess_noise = init_guess;
  init_guess_noise.translation().x() += translation_distribution_init(generator);
  init_guess_noise.translation().y() += translation_distribution_init(generator);
  init_guess_noise.translation().z() += translation_distribution_init(generator);
  // Apply random noise to the initial guess quaternion
  Sophus::SE3f::QuaternionType init_guess_quaternion = init_guess_noise.unit_quaternion();
  init_guess_quaternion.w() += quaternion_distribution_init(generator);
  init_guess_quaternion.x() += quaternion_distribution_init(generator);
  init_guess_quaternion.y() += quaternion_distribution_init(generator);
  init_guess_quaternion.z() += quaternion_distribution_init(generator);
  init_guess_quaternion.normalize();  // Ensure the quaternion remains valid
  init_guess_noise.setQuaternion(init_guess_quaternion);

  const float sigma = 6.0;
  const Sophus::SE3f T_icp = registration_->register_frame(
    transformed_frame, map_.get(), init_guess_noise, 3.0 * sigma, sigma / 3.0);

  std::cout << "Testing with solver_type: " << params.solver_type << std::endl;
  std::cout << "Iterations: " << registration_->get_debug().num_iter << std::endl;

  EXPECT_NEAR(T_icp.translation().x(), init_guess.translation().x(), 1.0e-1)
    << "Wrong translation in x-direction for " << params.string();
  EXPECT_NEAR(T_icp.translation().y(), init_guess.translation().y(), 1.0e-1)
    << "Wrong translation in y-direction for " << params.string();
  EXPECT_NEAR(T_icp.translation().z(), init_guess.translation().z(), 1.0e-1)
    << "Wrong translation in z-direction for " << params.string();
  EXPECT_NEAR(T_icp.unit_quaternion().w(), init_guess.unit_quaternion().w(), 1.0e-2)
    << "Wrong quaternion w-component for " << params.string();
  EXPECT_NEAR(T_icp.unit_quaternion().x(), init_guess.unit_quaternion().x(), 1.0e-2)
    << "Wrong quaternion x-component for " << params.string();
  EXPECT_NEAR(T_icp.unit_quaternion().y(), init_guess.unit_quaternion().y(), 1.0e-2)
    << "Wrong quaternion y-component for " << params.string();
  EXPECT_NEAR(T_icp.unit_quaternion().z(), init_guess.unit_quaternion().z(), 1.0e-2)
    << "Wrong quaternion z-component for " << params.string();
}
