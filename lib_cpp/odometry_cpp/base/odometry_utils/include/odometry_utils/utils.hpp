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

#include <algorithm>
#include <cstdio>
#include <fstream>
#include <iostream>
#include <memory>
#include <sstream>
#include <string>
#include <vector>

#ifdef USE_VISUALIZATION
#include <rerun.hpp>
#include <tuple>
#include <unordered_map>
#endif

#include <sophus/se3.hpp>

#include "odometry_types/odometry_config.hpp"
#include "odometry_utils/colors.hpp"

// Macro to initialize modules from config
#define INIT_MODULE(PARAM, TYPE, NAME, MODULE, CONFIG, DEBUG)                           \
  if constexpr (TConfig::PARAM == types::TYPE) {                                        \
    this->NAME = MODULE<TConfig>::from_config(CONFIG, DEBUG);                           \
    std::cout << "\033[1;36mInitialized module " << #MODULE << "!\033[0m" << std::endl; \
  }
// Macro to copy fields of different point types
// FLAG is the TConfig switch guarding the field, MEMBER the corresponding types::Point member.
// Both are needed because they differ (e.g. NORMALS guards `normal`); naming them with a single
// token silently skips the copy instead of failing to compile.
#define CONDITIONAL_COPY(FLAG, MEMBER)                        \
  if constexpr (TConfigSrc::FLAG && TConfigDst::FLAG) {       \
    dst.MEMBER = src.MEMBER;                                  \
  }
namespace tam::core::state::utils
{
/**
 * @brief Transform points with a given transformation
 * @param [in] T                  Transformation
 * @param [in] points             Points to transform
 */
template <typename TConfig>
inline void transform_points(const Sophus::SE3f & T, std::vector<types::Point<TConfig>> & points)
{
  std::transform(points.cbegin(), points.cend(), points.begin(), [&](const auto & point) {
    types::Point<TConfig> pt = point;
    pt.pos = T * point.pos;
    // Transform normals and covariances if they exist
    if constexpr (TConfig::NORMALS) {
      pt.normal = T.rotationMatrix() * point.normal;
    }
    if constexpr (TConfig::COV) {
      pt.cov = T.rotationMatrix() * point.cov * T.rotationMatrix().transpose();
    }
    return pt;
  });
}
/**
 * @brief Copy function for different point types
 * @param [in] src                  Point to convert to different format
 */
template <typename TConfigSrc, typename TConfigDst>
inline types::Point<TConfigDst> convert_point(const types::Point<TConfigSrc> & src)
{
  types::Point<TConfigDst> dst;
  // Unconditional members of types::Point.
  dst.pos = src.pos;
  dst.timestamp = src.timestamp;
  CONDITIONAL_COPY(SPHERICAL, azimuth);
  CONDITIONAL_COPY(SPHERICAL, range);
  CONDITIONAL_COPY(SPHERICAL, elevation);
  CONDITIONAL_COPY(INTENSITY, intensity);
  CONDITIONAL_COPY(NORMALS, normal);
  CONDITIONAL_COPY(COV, cov);
  CONDITIONAL_COPY(SEG, seg);
  CONDITIONAL_COPY(VEL, vel);
  CONDITIONAL_COPY(RCS, rcs);
  CONDITIONAL_COPY(SNR, snr);
  CONDITIONAL_COPY(CONFIDENCE, confidence);
  CONDITIONAL_COPY(VEL_INTERVAL, vel_interval);
  CONDITIONAL_COPY(SENSOR_ID, sensor_id);
  return dst;
}
/**
 * @brief Load point cloud from binary file in KITTI format
 *
 * @param[in] path                - std::string:
 *                                  absolute path to file
 * @return std::vector<types::Point<TConfig>>:
 */
template <typename TConfig>
inline std::vector<types::Point<TConfig>> load_pointcloud_bin(const std::string & path)
{
  std::vector<types::Point<TConfig>> point_cloud;

  // Open file
  FILE * stream = fopen(path.c_str(), "rb");
  if (!stream) {
    throw std::logic_error("Failed to open file: " + path);
  }

  // Determine file size
  fseek(stream, 0, SEEK_END);
  size_t file_size = ftell(stream);
  rewind(stream);

  size_t num_floats = file_size / sizeof(float);
  if (num_floats % 4 != 0) {
    fclose(stream);
    throw std::logic_error("File size does not match expected format: " + path);
  }

  // Allocate buffer
  std::unique_ptr<float[]> data(new float[num_floats]);
  size_t read_count = fread(data.get(), sizeof(float), num_floats, stream);
  fclose(stream);

  // Check if the correct number of floats was read
  if (read_count != num_floats) {
    throw std::runtime_error("Failed to read the expected number of floats from file: " + path);
  }

  // Parse points
  for (size_t i = 0; i < num_floats; i += 4) {
    types::Point<TConfig> pt;
    pt.pos.x() = data[i];
    pt.pos.y() = data[i + 1];
    pt.pos.z() = data[i + 2];
    point_cloud.push_back(pt);
  }

  return point_cloud;
}
/**
 * @brief Load pose from text file in format x y z qw qx qy qz x_stddev y_stddev z_stddev
 *
 * @param[in] path                - std::string:
 *                                  absolute path to file
 * @return Sophus::SE3f:          - pose
 */
template <typename TConfig>
inline Sophus::SE3f load_pose(const std::string & path)
{
  Sophus::SE3f pose;
  double x, y, z, qw, qx, qy, qz, x_stddev, y_stddev, z_stddev;

  std::ifstream infile(path);
  std::string line;

  while (std::getline(infile, line)) {
    std::istringstream iss(line);

    if (!(iss >> x >> y >> z >> qw >> qx >> qy >> qz >> x_stddev >> y_stddev >> z_stddev)) {
      std::cerr << "Pose in wrong format!" << std::endl;
      return pose;
    }
    pose = Sophus::SE3f(Sophus::SE3f::QuaternionType(qw, qx, qy, qz), Sophus::SE3f::Point(x, y, z));
  }
  return pose;
}
// Rerun specific functions for visualization
#ifdef USE_VISUALIZATION
/**
 * @brief Spawn a rerun recording stream
 * @param[in] name                - std::string:
 *                                 name of the recording stream
 * @return rerun::RecordingStream - recording stream
 */
inline rerun::RecordingStream spawn_rerun_stream(const std::string & name)
{
  rerun::RecordingStream rec = rerun::RecordingStream(name);
  rec.spawn().exit_on_failure();
  return rec;
}
/**
 * @brief Convert point cloud positions to rerun format
 * @param[in] points              - std::vector<types::Point<TConfig>>:
 *                                point cloud
 * @param[in] color               - std::string:
 *                               color of the points
 * @param[in] T                   - Sophus::SE3f:
 *                               transformation applied to the points
 * @return std::tuple<std::vector<rerun::Position3D>, std::vector<rerun::Color>>:
 *        positions, colors
 */
template <typename TConfig>
inline std::tuple<std::vector<rerun::Position3D>, std::vector<rerun::Color>> points2rerun(
  const std::vector<types::Point<TConfig>> & points, const std::string & color,
  const Sophus::SE3f & T = Sophus::SE3f())
{
  std::vector<rerun::Position3D> positions{};
  std::vector<rerun::Color> colors{};
  TUMcolor tum_color(color);
  positions.reserve(points.size());
  colors.reserve(points.size());
  for (const auto & point : points) {
    Eigen::Vector3f pos = (T * point.pos).template cast<float>();
    positions.push_back(rerun::Position3D(pos.x(), pos.y(), pos.z()));
    colors.push_back(rerun::Color(tum_color.r, tum_color.g, tum_color.b, tum_color.a));
  }
  return std::make_tuple(positions, colors);
}
/**
 * @brief Convert point cloud positions with normals to rerun format
 * @param[in] points              - std::vector<types::Point<TConfig>>:
 *                                point cloud
 * @param[in] color               - std::string:
 *                              color of the points
 * @return std::tuple<std::vector<rerun::Position3D>, std::vector<rerun::Color>>:
 *        positions, colors
 */
template <typename TConfig>
inline std::tuple<
  std::vector<rerun::Position3D>, std::vector<rerun::Color>, std::vector<rerun::Vector3D>>
points_normals2rerun(
  const std::vector<types::Point<TConfig>> & points,
  const std::string & color) requires types::HASNORMALCOV<TConfig>
{
  std::vector<rerun::Position3D> positions{};
  std::vector<rerun::Vector3D> normals{};
  std::vector<rerun::Color> colors{};
  TUMcolor tum_color(color);
  positions.reserve(points.size());
  normals.reserve(points.size());
  colors.reserve(points.size());
  for (const auto & point : points) {
    Eigen::Vector3f pos = point.pos.template cast<float>();
    Eigen::Vector3f normal = point.normal.template cast<float>();
    positions.push_back(rerun::Position3D(pos.x(), pos.y(), pos.z()));
    normals.push_back(rerun::Vector3D(normal.x(), normal.y(), normal.z()));
    colors.push_back(rerun::Color(tum_color.r, tum_color.g, tum_color.b, tum_color.a));
  }
  return std::make_tuple(positions, colors, normals);
}
#endif
}  // namespace tam::core::state::utils
