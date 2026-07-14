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

#include <iostream>
#include <vector>
#include <string>

#include "test_gicp_parameterized.hpp"
/**
 * @brief Test construction of GICP from param manager and logger
 */
TEST(GICP, BuildPmgLogger)
{
  // Initialize ICP
  // clang-format off
  // Construct from param manager and logger
  tam::pmg::ParamReferenceManager::UniquePtr pmg_ =  std::make_unique<tam::pmg::ParamReferenceManager>(); // NOLINT
  tam::tsl::ReferenceLogger::UniquePtr logger_ = std::make_unique<tam::tsl::ReferenceLogger>(); // NOLINT
  auto registration_ = tam::core::state::GICP<tam::core::state::types::GICP_EXT>::from_config(pmg_.get(), logger_.get()); // NOLINT
  // clang-format on

  tam::pmg::MgmtInterface * pmg_raw = pmg_.get();
  pmg_raw->set_value("registration.max_iter", 100);

  EXPECT_EQ(registration_->get_config().max_iter, 100)
    << "Failed to construct ICP from param manager and logger";
}
/**
 * @brief Test construction of GICP from config and debug object
 */
TEST(GICP, BuildConfigDebug)
{
  // clang-format off
  // Construct from config and debug objects
  tam::core::state::types::RegistrationConfig config;
  tam::core::state::types::RegistrationDebug debug;
  config.max_iter = 500;
  config.max_time = 150.0;
  config.convergence_criterion = 5e-3;
  auto registration_ = tam::core::state::GICP<tam::core::state::types::GICP_EXT>::from_config(config, debug); // NOLINT
  // clang-format on
  EXPECT_EQ(registration_->get_config().max_iter, 500)
    << "Failed to construct ICP from config and debug objects";
}
/**
 * @brief Instantiate tests with various parameter combinations
 */
INSTANTIATE_TEST_SUITE_P(
  GICPCombinations, GICPParameterizedTest,
  ::testing::Values(
    TestParams{"SVD", "GaussNewton"}, TestParams{"SVD", "LevenbergMarquardt"},
    TestParams{"MIN_EIGENVALUE", "GaussNewton"}, TestParams{"MIN_EIGENVALUE", "LevenbergMarquardt"},
    TestParams{"FROBENIUS", "GaussNewton"}, TestParams{"FROBENIUS", "LevenbergMarquardt"}),
  [](const ::testing::TestParamInfo<TestParams> & info) { return info.param.string(); });
