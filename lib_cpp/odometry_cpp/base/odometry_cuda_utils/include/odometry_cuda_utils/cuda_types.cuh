/*
 * Copyright 2024 Maximilian Leitenstern, Marcel Weinmann
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

#include <cuda_runtime.h>

#include <Eigen/Core>

#include "odometry_types/point_types.hpp"

namespace tam::core::state::cuda {
// Constants
static constexpr int BLOCK_SIZE = 256;
// Warp size for parallel reduction
static constexpr int WARP_SIZE = 32;
// Size of linear system (6x6 JTJ, 6x1 JTr)
static constexpr int LS_SIZE = 42;

/**
 * @brief Voxel type for CUDA implementation
 */
struct alignas(8) custom_voxel_type {
  int16_t x;
  int16_t y;
  int16_t z;
  int16_t padding = 0;

  __host__ __device__ custom_voxel_type() {}

  __host__ __device__ custom_voxel_type(int16_t padding) : padding{padding} {}

  __host__ __device__ custom_voxel_type(int16_t x, int16_t y, int16_t z) : x{x}, y{y}, z{z} {}

  __host__ __device__ bool operator==(const custom_voxel_type& other) const
  {
    return x == other.x && y == other.y && z == other.z && padding == other.padding;
  }
};

/**
 * @brief Array of points for each voxel
 * @brief Needs to know the max amount of points per voxel at compile time for
 *        preallocation
 */
template <typename TConfig>
struct alignas(16) fixed_sized_points_array {
  int32_t counter = 0;
  int lock = 0;
  types::Point<TConfig> data[TConfig::MAX_POINTS_PER_VOXEL];
};
}  // namespace tam::core::state::cuda
