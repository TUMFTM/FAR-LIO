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

#include "odometry_types/odometry_types.hpp"
#include "odometry_types/point_types.hpp"
#include "robust_kernel/robust_kernel.hpp"
/**
 * @brief Test Huber kernel
 */
// TEST(RobustKernel, Huber)
// {
//   // To be implemented -> define config using Huber Kernel
// }

/**
 * @brief Test Cauchy kernel
 */
// TEST(RobustKernel, Cauchy)
// {
//   // To be implemented -> define config using Cauchy Kernel
// }

/**
 * @brief Test GemanMcClure kernel
 */
TEST(RobustKernel, GemanMcClure)
{
  // Define residual and kernel scale
  float residual = 0.5;
  float kernel_scale = 1.5;

  // Compute the weight of the residual using the GemanMcClure kernel
  // clang-format off
  float weight_gemanmcclure = tam::core::state::robust_kernel_weight<tam::core::state::types::ICP_EXT>(Eigen::Vector3f(sqrt(residual), 0.0f, 0.0f), kernel_scale); // NOLINT
  // clang-format on

  EXPECT_NEAR(weight_gemanmcclure, 0.5625, 1.0e-3) << "Wrong weight for GemanMcClure kernel";
}
