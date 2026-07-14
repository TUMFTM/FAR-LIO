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
#include "velocity_handler/derivative.hpp"

#include <gtest/gtest.h>

#include <chrono>
#include <memory>

#include "param_management_cpp/param_reference_manager.hpp"
#include "tsl_logger_cpp/reference_logger.hpp"
/**
 * @brief First call has no buffered pose -> zero tangent.
 */
TEST(Derivative, FirstCallReturnsZero)
{
  tam::core::state::types::VelocityConfig config;
  tam::core::state::types::VelocityDebug debug;
  auto vh =
    tam::core::state::Derivative<tam::core::state::types::ICP_EXT>::from_config(config, debug);

  tam::core::state::types::PoseStamped pose;
  pose.stamp = static_cast<std::uint64_t>(1e9);
  const auto result = vh->get_tangent({}, pose);
  EXPECT_EQ(result.tangent.norm(), 0.0f);
  EXPECT_EQ(result.stamp, pose.stamp);
}
/**
 * @brief Translation by 1 m over 0.1 s along x -> 10 m/s linear x velocity.
 */
TEST(Derivative, ConstantTranslation)
{
  tam::core::state::types::VelocityConfig config;
  tam::core::state::types::VelocityDebug debug;
  auto vh =
    tam::core::state::Derivative<tam::core::state::types::ICP_EXT>::from_config(config, debug);

  tam::core::state::types::PoseStamped p0;
  p0.stamp = static_cast<std::uint64_t>(1e9);
  vh->get_tangent({}, p0);  // prime buffer

  tam::core::state::types::PoseStamped p1;
  p1.pose.translation().x() = 1.0f;
  p1.stamp = static_cast<std::uint64_t>(1.1e9);
  const auto result = vh->get_tangent({}, p1);
  EXPECT_NEAR(result.tangent(0), 10.0f, 1e-3);
  EXPECT_NEAR(result.tangent(1), 0.0f, 1e-3);
  EXPECT_NEAR(result.tangent(2), 0.0f, 1e-3);
}
/**
 * @brief A dt > 1 s gap resets velocity to zero.
 */
TEST(Derivative, LargeTimeGapResetsVelocity)
{
  tam::core::state::types::VelocityConfig config;
  tam::core::state::types::VelocityDebug debug;
  auto vh =
    tam::core::state::Derivative<tam::core::state::types::ICP_EXT>::from_config(config, debug);

  tam::core::state::types::PoseStamped p0;
  p0.stamp = static_cast<std::uint64_t>(1e9);
  vh->get_tangent({}, p0);

  tam::core::state::types::PoseStamped p1;
  p1.pose.translation().x() = 50.0f;
  p1.stamp = static_cast<std::uint64_t>(3e9);  // 2 s later
  const auto result = vh->get_tangent({}, p1);
  EXPECT_EQ(result.tangent.norm(), 0.0f);
}
