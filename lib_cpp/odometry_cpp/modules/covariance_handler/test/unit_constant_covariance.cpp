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

#include "covariance_handler/constant_covariance.hpp"
#include "param_management_cpp/param_reference_manager.hpp"
#include "tsl_logger_cpp/reference_logger.hpp"

/**
 * @brief Test construction of ConstantCov from param manager and logger
 */
TEST(ConstantCov, BuildPmgLogger)
{
  // clang-format off
  // Construct from param manager and logger
  tam::pmg::ParamReferenceManager::UniquePtr pmg_ =  std::make_unique<tam::pmg::ParamReferenceManager>(); // NOLINT
  tam::tsl::ReferenceLogger::UniquePtr logger_ = std::make_unique<tam::tsl::ReferenceLogger>(); // NOLINT
  std::unique_ptr<tam::core::state::CovarianceHandler<tam::core::state::types::ICP_EXT>> cov_ = // NOLINT
    tam::core::state::ConstantCovariance<tam::core::state::types::ICP_EXT>::from_config(pmg_.get(), logger_.get()); // NOLINT
  // clang-format on

  tam::pmg::MgmtInterface* pmg_raw = pmg_.get();
  pmg_raw->set_value("covariance.min_cov_translation", std::vector<double>{0.05, 0.05, 0.05});

  EXPECT_EQ(cov_->get_config().min_cov_translation[0], 0.05)
    << "Failed to construct ConstantCovariance from param manager and logger";
}

/**
 * @brief Test construction of ConstantCov from config and debug object
 */
TEST(ConstantCov, BuildConfigDebug)
{
  // clang-format off
  // Construct from config and debug objects
  tam::core::state::types::CovarianceConfig config;
  tam::core::state::types::CovarianceDebug debug;
  std::unique_ptr<tam::core::state::CovarianceHandler<tam::core::state::types::ICP_EXT>> cov_ = // NOLINT
    tam::core::state::ConstantCovariance<tam::core::state::types::ICP_EXT>::from_config(config, debug); // NOLINT
  // clang-format on

  cov_->get_config().min_cov_translation[0] = 0.05;
  EXPECT_EQ(cov_->get_config().min_cov_translation[0], 0.05)
    << "Failed to construct ConstantCovariance from param config and debug";
}
