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
// based on https://github.com/PRBonn/kiss-icp/blob/main/cpp/kiss_icp/core/VoxelHashMap.hpp
#pragma once

#include <tsl/robin_map.h>

#include <eigen3/Eigen/Core>
#include <memory>
#include <string>
#include <tuple>
#include <vector>

#include "odometry_types/point_types.hpp"

// Required as tsl namespace collides with tam::tsl namespace
using tsl::robin_map;

namespace tam::core::state {
// Voxel definition as Eigen::Vector3i
using Voxel = Eigen::Vector3i;

/**
 * @brief Convert a point to a voxel
 * @param [in] point        Point to convert
 * @param [in] voxel_size   Size of the voxel
 */
template <typename TConfig>
inline Voxel point_to_voxel(const types::Point<TConfig>& point, const double voxel_size)
{
  return Voxel(static_cast<int>(std::floor(point.pos.x() / voxel_size)),
    static_cast<int>(std::floor(point.pos.y() / voxel_size)), static_cast<int>(std::floor(point.pos.z() / voxel_size)));
}

/**
 * @brief Get the adjacent voxels of a voxel
 * @param [in] voxel               Voxel to get neighbors of
 * @param [in] adjacent_voxels     Number of adjacent voxels to get
 * @return                         Vector of adjacent voxels
 */
inline std::vector<Voxel> get_adjacent_voxels(const Voxel& voxel, int adjacent_voxels = 1)
{
  std::vector<Voxel> voxel_neighborhood;
  voxel_neighborhood.reserve(27);
  for (int i = voxel.x() - adjacent_voxels; i < voxel.x() + adjacent_voxels + 1; ++i) {
    for (int j = voxel.y() - adjacent_voxels; j < voxel.y() + adjacent_voxels + 1; ++j) {
      for (int k = voxel.z() - adjacent_voxels; k < voxel.z() + adjacent_voxels + 1; ++k) {
        voxel_neighborhood.emplace_back(i, j, k);
      }
    }
  }
  return voxel_neighborhood;
}

/**
 * @brief Downsample a frame of points to a voxel grid
 * => Only one point per voxel
 * @param [in] frame        Frame of points to downsample
 * @param [in] voxel_size   Size of the voxel
 * @return Downsampled frame
 */
template <typename TConfig>
std::vector<types::Point<TConfig>> voxel_downsample(
  const std::vector<types::Point<TConfig>>& frame, const double voxel_size)
{
  robin_map<Voxel, types::Point<TConfig>> grid;
  grid.reserve(frame.size());
  std::for_each(frame.cbegin(), frame.cend(), [&](const auto& point) {
    const Voxel vox = point_to_voxel<TConfig>(point, voxel_size);
    if (!grid.contains(vox)) grid.insert({vox, point});
  });
  std::vector<types::Point<TConfig>> frame_dowsampled;
  frame_dowsampled.reserve(grid.size());
  std::for_each(grid.cbegin(), grid.cend(),
    [&](const auto& voxel_and_point) { frame_dowsampled.emplace_back(voxel_and_point.second); });
  return frame_dowsampled;
}

/**
 * @brief Downsample a frame of points to a voxel grid twice with different
 * voxel sizes
 * @param [in] frame        Frame of points to downsample
 * @param [in] voxel_size   Size of the voxel for first downsampling
 * @return Downsampled frame
 */
template <typename TConfig>
std::tuple<std::vector<types::Point<TConfig>>, std::vector<types::Point<TConfig>>> voxel_doubledownsample(
  const std::vector<types::Point<TConfig>>& frame, const double voxel_size)
{
  const std::vector<types::Point<TConfig>> frame_0_5 = voxel_downsample(frame, voxel_size * 0.5);
  const std::vector<types::Point<TConfig>> frame_1_5 = voxel_downsample(frame_0_5, voxel_size * 1.5);
  return std::make_tuple(frame_1_5, frame_0_5);
}
}  // namespace tam::core::state

/**
 * @brief Hash function for Voxel
 */
template <>
struct std::hash<tam::core::state::Voxel> {
  std::size_t operator()(const tam::core::state::Voxel& voxel) const
  {
    const uint32_t* vec = reinterpret_cast<const uint32_t*>(voxel.data());
    return (vec[0] * 73856093 ^ vec[1] * 19349669 ^ vec[2] * 83492791);
  }
};
