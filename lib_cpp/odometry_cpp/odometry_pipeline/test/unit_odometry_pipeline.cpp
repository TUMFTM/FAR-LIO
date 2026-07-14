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

#include "odometry_pipeline/odometry_pipeline.hpp"
#include "param_management_cpp/param_reference_manager.hpp"
#include "tsl_logger_cpp/reference_logger.hpp"
/**
 * @brief Test construction of OdometryPipeline from param manager and logger
 */
TEST(OdometryPipeline, BuildPmgLogger)
{
  // clang-format off
  // Construct from param manager and logger
  tam::pmg::ParamReferenceManager::UniquePtr pmg_ =  std::make_unique<tam::pmg::ParamReferenceManager>(); // NOLINT
  tam::tsl::ReferenceLogger::UniquePtr logger_ = std::make_unique<tam::tsl::ReferenceLogger>(); // NOLINT

  std::unique_ptr<tam::core::state::OdometryPipeline<tam::core::state::types::ICP_EXT>> odom_pipeline =  // NOLINT
    tam::core::state::OdometryPipeline<tam::core::state::types::ICP_EXT>::from_config(pmg_.get(), logger_.get()); // NOLINT
  // clang-format on

  // Check correct parameterization based on single parameter
  tam::pmg::MgmtInterface * pmg_raw = pmg_.get();
  pmg_raw->set_value("diagnostic.frame_outdated", 97.0);  // NOLINT
  EXPECT_EQ(odom_pipeline->get_diagnostic_config().frame_outdated, 97.0);
}
/**
 * @brief Test construction of OdometryPipeline from config and debug object
 */
TEST(OdometryPipeline, BuildConfigDebug) {
    // Construct from config and debug object
    tam::core::state::types::PipelineConfig config;
    tam::core::state::types::PipelineDebug debug;
    // Check correct parameterization based on single parameter

    std::unique_ptr<tam::core::state::OdometryPipeline<tam::core::state::types::ICP_EXT>> odom_pipeline =  // NOLINT
        tam::core::state::OdometryPipeline<tam::core::state::types::ICP_EXT>::from_config(config, debug); // NOLINT

    // Check correct parameterization based on single parameter
    odom_pipeline->get_diagnostic_config().frame_outdated = 97.0;

    EXPECT_EQ(odom_pipeline->get_diagnostic_config().frame_outdated, 97.0);
}
