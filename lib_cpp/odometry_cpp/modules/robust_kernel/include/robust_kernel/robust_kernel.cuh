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
#pragma once

#include "odometry_types/odometry_config.hpp"
#include "odometry_types/odometry_types.hpp"
#include "odometry_types/point_types.hpp"

namespace tam::core::state::cuda {
struct Square {
  __host__ __device__ float operator()(float x) const { return x * x; }
};

/**
 * @brief Calculate the Kernel weight
 * @param [in] residual       Square norm of the residual between the correspondences
 * @param [in] kernel_scale   Scale of the kernel
 */
template <typename TConfig>
struct RobustKernelWeight {
  __host__ __device__ float operator()(const Eigen::Vector3f& residual, float kernel_scale) const
  {
    // clang-format off
    if constexpr (TConfig::KERNEL == tam::core::state::types::RobustKernelType::HUBER) {
      // Huber kernel weight from small_gicp
      // (https://github.com/koide3/small_gicp/blob/master/include/small_gicp/factors/robust_kernel.hpp)
      const float res_abs = abs(residual.squaredNorm());
      return (res_abs < kernel_scale) ? 1.0 : kernel_scale / res_abs;
    } else if constexpr (TConfig::KERNEL == tam::core::state::types::RobustKernelType::CAUCHY) {
      // Cauchy kernel weight (P. Babin, P. Giguere, and F. Pomerleau,
      // “Analysis of robust functions for registration algorithms”)
      // in 2019 International Conference on Robotics and Automation (ICRA))
      return 1.0f / (1.0f + (residual / kernel_scale).squaredNorm());
    } else if constexpr (TConfig::KERNEL == tam::core::state::types::RobustKernelType::GEMANMCCLURE) {  // NOLINT
      // Geman-McClure kernel weight from kiss-icp
      // (https://github.com/PRBonn/kiss-icp/blob/main/cpp/kiss_icp/core/Registration.cpp)
      return Square()(kernel_scale) / Square()(kernel_scale + residual.squaredNorm());
    } else if constexpr (TConfig::KERNEL == tam::core::state::types::RobustKernelType::WELSCH) {
      // Welsch kernel weight(P. Babin, P. Giguere, and F. Pomerleau,
      // “Analysis of robust functions for registration algorithms”)
      // in 2019 International Conference on Robotics and Automation (ICRA))
      return expf(-(residual / kernel_scale).squaredNorm());
    }
    // clang-format on
    return 1.0;
  }
};
}  // namespace tam::core::state::cuda
