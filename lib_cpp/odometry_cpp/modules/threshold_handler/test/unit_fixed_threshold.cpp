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

#include "threshold_handler/fixed_threshold.hpp"
#include "param_management_cpp/param_reference_manager.hpp"
#include "tsl_logger_cpp/reference_logger.hpp"
/**
 * @brief Test construction of FixedThreshold from param manager and logger
 */
TEST(FixedThreshold, BuildPmgLogger)
{
  // clang-format off
  // Construct from param manager and logger
  tam::pmg::ParamReferenceManager::UniquePtr pmg_ =  std::make_unique<tam::pmg::ParamReferenceManager>(); // NOLINT
  tam::tsl::ReferenceLogger::UniquePtr logger_ = std::make_unique<tam::tsl::ReferenceLogger>(); // NOLINT
  std::unique_ptr<tam::core::state::ThresholdHandler<tam::core::state::types::ICP_EXT>> threshold_ =  // NOLINT
    tam::core::state::FixedThreshold<tam::core::state::types::ICP_EXT>::from_config(pmg_.get(), logger_.get()); // NOLINT
  // clang-format on
  tam::pmg::MgmtInterface * pmg_raw = pmg_.get();
  pmg_raw->set_value("threshold.initial_threshold", 8.0);
  EXPECT_EQ(threshold_->get_config().initial_threshold, 8.0)
    << "Failed to construct FixedThreshold from param manager and logger";
}
/**
 * @brief Test construction of FixedThreshold from config and debug object
 */
TEST(FixedThreshold, BuildConfigDebug)
{
  // clang-format off
  // Construct from config and debug objects
  tam::core::state::types::ThresholdConfig config;
  tam::core::state::types::ThresholdDebug debug;
  config.initial_threshold = 8.0;
  config.min_motion_threshold = 3.0;
  config.max_correspondence_range = 2.0;
  std::unique_ptr<tam::core::state::ThresholdHandler<tam::core::state::types::ICP_EXT>> threshold_ =  // NOLINT
    tam::core::state::FixedThreshold<tam::core::state::types::ICP_EXT>::from_config(config, debug); // NOLINT
  // clang-format on
  EXPECT_EQ(threshold_->get_config().initial_threshold, 8.0)
    << "Failed to construct FixedThreshold from config and debug";
}

