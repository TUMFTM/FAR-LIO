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

#include "odometry_types/odometry_config.hpp"
#include "odometry_types/point_types.hpp"
#include "preprocessing_handler/lidar_preprocessing.hpp"

/**
 * @brief Test construction of LidarPreprocessing from param manager and logger
 */
TEST(LidarPreprocessing, BuildPmgLogger)
{
  // clang-format off
  // Construct from param manager and logger
  tam::pmg::ParamReferenceManager::UniquePtr pmg_ =  std::make_unique<tam::pmg::ParamReferenceManager>(); // NOLINT
  tam::tsl::ReferenceLogger::UniquePtr logger_ = std::make_unique<tam::tsl::ReferenceLogger>(); // NOLINT
  auto lidar_preprocessing_ = tam::core::state::LidarPreprocessing<tam::core::state::types::ICP_EXT>::from_config(pmg_.get(), logger_.get()); // NOLINT
  // clang-format on
  tam::pmg::MgmtInterface* pmg_raw = pmg_.get();
  pmg_raw->set_value("preprocessing.crop_range", std::vector<double>{5.0, 50.0});
  EXPECT_EQ(lidar_preprocessing_->get_config().crop_range.front(), 5.0)
    << "Failed to construct LidarPreprocessing from param manager and logger";
}

/**
 * @brief Test construction of LidarPreprocessing from config and debug object
 */
TEST(LidarPreprocessing, BuildConfigDebug)
{
  // clang-format off
  // Construct from config and debug objects
  tam::core::state::types::PreprocessingConfig config;
  tam::core::state::types::PreprocessingDebug debug;
  config.crop_range = std::vector<double>{5.0, 50.0};
  auto lidar_preprocessing_ = tam::core::state::LidarPreprocessing<tam::core::state::types::ICP_EXT>::from_config(config, debug); // NOLINT
  // clang-format on
  EXPECT_EQ(lidar_preprocessing_->get_config().crop_range.front(), 5.0)
    << "Failed to construct LidarPreprocessing from config and debug";
}
