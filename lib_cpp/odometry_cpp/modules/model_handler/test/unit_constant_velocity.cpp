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
#include <memory>

#include "model_handler/constant_velocity.hpp"
#include "param_management_cpp/param_reference_manager.hpp"
#include "tsl_logger_cpp/reference_logger.hpp"
/**
 * @brief Test construction of ConstantVelocity from param manager and logger
 */
TEST(ConstantVelocity, BuildPmgLogger)
{
  // clang-format off
  // Construct from param manager and logger
  tam::pmg::ParamReferenceManager::UniquePtr pmg_ =  std::make_unique<tam::pmg::ParamReferenceManager>(); // NOLINT
  tam::tsl::ReferenceLogger::UniquePtr logger_ = std::make_unique<tam::tsl::ReferenceLogger>(); // NOLINT
  std::unique_ptr<tam::core::state::ModelHandler<tam::core::state::types::ICP_CV>> model_ =
    tam::core::state::ConstantVelocity<tam::core::state::types::ICP_CV>::from_config(pmg_.get(), logger_.get()); // NOLINT
  // clang-format on
  tam::pmg::MgmtInterface * pmg_raw = pmg_.get();
  pmg_raw->set_value("model.initial_pos_z", 2.0);
  model_->init_model_config();
  EXPECT_EQ(model_->get_initial_guess(std::uint64_t{0}).pose.translation().z(), 2.0)
    << "Failed to construct ConstantVelocity from param manager and logger";
}
/**
 * @brief Test construction of ConstantVelocity from config and debug object
 */
TEST(ConstantVelocity, BuildConfigDebug)
{
  // clang-format off
  // Construct from config and debug objects
  tam::core::state::types::ModelConfig config;
  tam::core::state::types::ModelDebug debug;
  config.initial_pos_x = 1.0;
  config.initial_pos_y = 3.0;
  config.initial_pos_z = 2.0;
  config.initial_rot_x = 1.0;
  config.initial_rot_y = 2.0;
  config.initial_rot_z = 3.0;
  config.initial_rot_w = 4.0;
  std::unique_ptr<tam::core::state::ModelHandler<tam::core::state::types::ICP_CV>> model_ =
    tam::core::state::ConstantVelocity<tam::core::state::types::ICP_CV>::from_config(config, debug); // NOLINT
  model_->init_model_config();

  // clang-format on
  EXPECT_EQ(model_->get_initial_guess(std::uint64_t{0}).pose.translation().z(), 2.0)
    << "Failed to construct ConstantVelocity from config and debug object";
}
/**
 * @brief Test prediction of ConstantVelocity
 */
TEST(ConstantVelocityPrediction, BuildConfigDebug)
{
  // clang-format off
  // Construct from param manager and logger
  tam::pmg::ParamReferenceManager::UniquePtr pmg_ =  std::make_unique<tam::pmg::ParamReferenceManager>(); // NOLINT
  tam::tsl::ReferenceLogger::UniquePtr logger_ = std::make_unique<tam::tsl::ReferenceLogger>(); // NOLINT
  std::unique_ptr<tam::core::state::ModelHandler<tam::core::state::types::ICP_CV>> model_ =
    tam::core::state::ConstantVelocity<tam::core::state::types::ICP_CV>::from_config(pmg_.get(), logger_.get()); // NOLINT
  // clang-format on
  // Default is position 0 and rotation 0
  model_->init_model_config();
  std::uint64_t stamp = std::chrono::system_clock::now().time_since_epoch().count();
  Sophus::SE3f pose = Sophus::SE3f();
  pose.translation() = Eigen::Vector3f(1.0, 2.0, 0.0);
  model_->set_pose(
    tam::core::state::types::PoseStamped{pose, stamp}, true);  // NOLINT
  EXPECT_EQ(
    model_->get_initial_guess(std::uint64_t{0}).pose.translation(), Eigen::Vector3f(2.0, 4.0, 0.0))
    << "Failed to get correct prediction from constant velocity model from config and debug object";
}
