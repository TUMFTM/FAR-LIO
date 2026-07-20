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

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <algorithm>
#include <cstdint>
#include <filesystem>  // NOLINT
#include <fstream>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/transform.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <iostream>
#include <memory>
#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <sophus/se3.hpp>
#include <sstream>
#include <std_msgs/msg/header.hpp>
#include <string>
#include <vector>

#include "odometry_types/odometry_types.hpp"
#include "odometry_types/point_types.hpp"
#include "odometry_types/odometry_config.hpp"
#include "ros2_watchdog_cpp/node_monitor.hpp"
//
// Macros for optional PointCloud2 fields handled by cloud2eigen / eigen2cloud.
// F(member, c++-type, ros_datatype_suffix, write_name, read_name_aliases...)
#define SPHERICAL_FIELDS(F)                        \
  F(azimuth, float, FLOAT32, "azimuth", "azimuth") \
  F(range, float, FLOAT32, "range", "range")       \
  F(elevation, float, FLOAT32, "elevation", "elevation")
#define INTENSITY_FIELDS(F) F(intensity, float, FLOAT32, "intensity", "intensity")
#define RADAR_FIELDS(F)                                     \
  F(vel, float, FLOAT32, "velocity", "velocity")            \
  F(rcs, float, FLOAT32, "rcs", "rcs")                      \
  F(snr, float, FLOAT32, "snr", "snr")                      \
  F(confidence, float, FLOAT32, "confidence", "confidence") \
  F(vel_interval, float, FLOAT32, "vel_interval", "velocity_interval")
#define SENSOR_ID_FIELDS(F) F(sensor_id, std::uint8_t, UINT8, "sensor_id", "sensor_id", "id")
// Read Macros
#define DECL_CONST_ITER(member, type, dtype, wname, ...) \
  std::optional<sensor_msgs::PointCloud2ConstIterator<type>> it_##member;
#define INIT_CONST_ITER(member, type, dtype, wname, ...) \
  it_##member.emplace(msg, tam::core::state::utils::find_field(msg, sensor_msgs::msg::PointField::dtype, __VA_ARGS__));
// Validation entry: contributes a `|| <missing>` term for use in chained checks.
#define CHECK_FIELD(member, type, dtype, wname, ...) \
  || tam::core::state::utils::find_field(msg, sensor_msgs::msg::PointField::dtype, __VA_ARGS__).empty()
#define READ_ITER(member, type, dtype, wname, ...) \
  points[i].member = **it_##member;                \
  ++(*it_##member);
// Write Macros
#define DECL_ITER(member, type, dtype, wname, ...) std::optional<sensor_msgs::PointCloud2Iterator<type>> it_##member;
#define INIT_ITER(member, type, dtype, wname, ...) it_##member.emplace(cloud_msg, wname);
#define WRITE_ITER(member, type, dtype, wname, ...) \
  **it_##member = points[i].member;                 \
  ++(*it_##member);
#define ADD_FIELD(member, type, dtype, wname, ...) \
  offset = addPointField(cloud_msg, wname, 1, sensor_msgs::msg::PointField::dtype, offset);

namespace tam::core::state {
namespace utils {
/**
 * @brief Convert a ROS time stamp (sec/nanosec) to nanoseconds
 * @param[in] stamp              ROS time stamp with `.sec` and `.nanosec` members
 * @return timestamp in nanoseconds
 */
template <typename TimeT>
inline std::uint64_t stamp2ns(const TimeT& stamp)
{
  return static_cast<std::uint64_t>(stamp.sec) * 1'000'000'000ULL + static_cast<std::uint64_t>(stamp.nanosec);
}

/**
 * @brief transform sophus pose to geometry msgs
 *
 * @param[in] T                  - Sophus::SE3f:
 * @return geometry_msgs::msg::Pose
 */
geometry_msgs::msg::Pose sophus2pose(const Sophus::SE3f& T)
{
  geometry_msgs::msg::Pose t;
  t.position.x = T.translation().x();
  t.position.y = T.translation().y();
  t.position.z = T.translation().z();

  Eigen::Quaternionf q(T.so3().unit_quaternion());
  t.orientation.x = q.x();
  t.orientation.y = q.y();
  t.orientation.z = q.z();
  t.orientation.w = q.w();
  return t;
}

/**
 * @brief transform sophus pose to geometry msgs
 * @param[in] T                  - Sophus::SE3f:
 */
geometry_msgs::msg::Transform sophus2transform(const Sophus::SE3f& T)
{
  geometry_msgs::msg::Transform t;
  t.translation.x = T.translation().x();
  t.translation.y = T.translation().y();
  t.translation.z = T.translation().z();

  Eigen::Quaternionf q(T.so3().unit_quaternion());
  t.rotation.x = q.x();
  t.rotation.y = q.y();
  t.rotation.z = q.z();
  t.rotation.w = q.w();
  return t;
}

/**
 * @brief Transform odometry message to PoseStamped
 * @param[in] odom               - nav_msgs::msg::Odometry::SharedPtr
 * @return types::PoseStamped
 */
types::PoseStamped odom2pose(const nav_msgs::msg::Odometry::SharedPtr& odom)
{
  types::PoseStamped pose;
  // Timestamp
  pose.stamp = stamp2ns(odom->header.stamp);
  // Pose
  pose.pose = Sophus::SE3f(Sophus::SE3f::QuaternionType(odom->pose.pose.orientation.w, odom->pose.pose.orientation.x,
                             odom->pose.pose.orientation.y, odom->pose.pose.orientation.z),
    Sophus::SE3f::Point(odom->pose.pose.position.x, odom->pose.pose.position.y, odom->pose.pose.position.z));
  return pose;
}

/**
 * @brief Transform odometry message to TangentStamped
 * @param[in] odom               - nav_msgs::msg::Odometry::SharedPtr
 * @return types::TangentStamped
 */
types::TangentStamped odom2tangent(const nav_msgs::msg::Odometry::SharedPtr& odom)
{
  types::TangentStamped tangent;
  // Timestamp
  tangent.stamp = odom->header.stamp.sec * 1e9 + odom->header.stamp.nanosec;
  // Tangent
  const auto& twist = odom->twist.twist;
  tangent.tangent << twist.linear.x, twist.linear.y, twist.linear.z, twist.angular.x, twist.angular.y, twist.angular.z;
  return tangent;
}

/**
 * @brief transform tf2 transform to Sophus pose
 * @param[in] transform          - geometry_msgs::msg::TransformStamped:
 *                                 input transform
 * @return types::PoseStamped

 */
types::PoseStamped transform2pose(const geometry_msgs::msg::TransformStamped& transform)
{
  types::PoseStamped pose;
  pose.stamp = stamp2ns(transform.header.stamp);
  const auto& t = transform.transform;
  pose.pose = Sophus::SE3f(Sophus::SE3f::QuaternionType(t.rotation.w, t.rotation.x, t.rotation.y, t.rotation.z),
    Sophus::SE3f::Point(t.translation.x, t.translation.y, t.translation.z));
  return pose;
}

/**
 * @brief Transform Odometry to nav_msgs::msg::Odometry message
 * @param[in] odom               - types::Odometry
 * @param[in] frame_id           - std::string
 *                                 frame id of the odometry message
 * @param[in] child_frame_id     - std::string
 *                                 child frame id of the odometry message
 * @return nav_msgs::msg::Odometry
 */
nav_msgs::msg::Odometry odom2msg(
  const types::Odometry& odom, const std::string& frame_id, const std::string& child_frame_id)
{
  nav_msgs::msg::Odometry msg;
  msg.header.stamp.sec = odom.stamp / 1000000000;
  msg.header.stamp.nanosec = odom.stamp % 1000000000;
  msg.header.frame_id = frame_id;
  msg.child_frame_id = child_frame_id;
  msg.pose.pose = sophus2pose(odom.pose.pose);
  msg.twist.twist.linear.x = odom.tangent.tangent[0];
  msg.twist.twist.linear.y = odom.tangent.tangent[1];
  msg.twist.twist.linear.z = odom.tangent.tangent[2];
  msg.twist.twist.angular.x = odom.tangent.tangent[3];
  msg.twist.twist.angular.y = odom.tangent.tangent[4];
  msg.twist.twist.angular.z = odom.tangent.tangent[5];
  std::transform(odom.pose.covariance.begin(), odom.pose.covariance.end(), msg.pose.covariance.begin(),
    [](float val) { return static_cast<double>(val); });
  std::transform(odom.tangent.covariance.begin(), odom.tangent.covariance.end(), msg.twist.covariance.begin(),
    [](float val) { return static_cast<double>(val); });
  return msg;
}

/**
 * @brief Transform diagnostic status to internal status
 * @param[in] msg                - diagnostic_msgs::msg::DiagnosticArray::ConstSharedPtr
 * @return types::DiagnosticStatus
 */
types::DiagnosticStatus convert_diag_status(const diagnostic_msgs::msg::DiagnosticArray::ConstSharedPtr& msg)
{
  types::DiagnosticStatus status{};
  // Take the first status message
  if (msg->status.empty()) {
    return status;
  }
  status.level = static_cast<types::DiagnosticLevel>(msg->status[0].level);
  status.message = msg->status[0].message;
  // NOTE: not converting key values here
  return status;
}

/**
 * @brief Set node monitor diagnostics
 * @param[in] odom               - types::Odometry
 * @param[in] monitor            - tam::core::NodeMonitor
 */
void set_monitor(const types::Odometry& odom, tam::core::NodeMonitor* monitor)
{
  // Set error level and message
  monitor->set_error_lvl("diagnostic_status", static_cast<tam::types::ErrorLvl>(odom.status.level));
  monitor->set_message(odom.status.message);
  // Report key values to dashboard
  for (const auto& [key, value] : odom.status.key_values) {
    monitor->report_value(key, value);
  }
}

/**
 * @brief load static point cloud map from pcd file
 * @param[in] file_path          - std::string full path to the .pcd file
 * @return std::vector<types::Point<TConfig>>
 */
template <typename TConfig>
std::vector<types::Point<TConfig>> load_static_map(const std::string& file_path)
{
  std::cout << "Load map from " << file_path << std::endl;

  pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(
    new pcl::PointCloud<pcl::PointXYZI>);
  std::vector<types::Point<TConfig>> map{};

  if (pcl::io::loadPCDFile<pcl::PointXYZI>(file_path, *cloud) == -1) {
    std::cout << "Couldn't read file " << file_path << std::endl;
    return map;
  }

  map.reserve(cloud->size());

  for (const auto& pt : cloud->points) {
    types::Point<TConfig> p;
    p.pos = Eigen::Vector3f{pt.x, pt.y, pt.z};
    // Assign intensity if available
    if constexpr (TConfig::INTENSITY) p.intensity = pt.intensity;
    map.emplace_back(p);
  }

  std::cout << "Loading completed: " << cloud->width * cloud->height << " points" << std::endl;
  return map;
}

/**
 * @brief Search for a field in a PointCloud2 msg, given a list of candidates
 *
 * @param[in] msg                - sensor_msgs::msg::PointCloud2
 *                                 msg to search in
 * @param[in] datatype           - std::uint8_t
 *                                 datatype of target field
 * @param[in] candidates         - template S
 *                                 list of candidates of field names
 */
template <typename... S>
inline std::string find_field(const sensor_msgs::msg::PointCloud2& msg, std::uint8_t datatype, S&&... candidates)
{
  for (const auto& f : msg.fields) {
    if (f.datatype != datatype) continue;
    if (((f.name == candidates) || ...)) return f.name;
  }
  return {};
}

/**
 * @brief transform incoming pointcloud to target frame and convert it to Eigen-vector
 *
 * @param[in] msg                - sensor_msgs::msg::PointCloud2
 *                                incoming frame
 * @param[in] iso                - Eigen::Isometry3f
 *                                transformation to apply
 * @return std::vector<types::Point<TConfig>>
 */
template <typename TConfig>
std::vector<types::Point<TConfig>> cloud2eigen(const sensor_msgs::msg::PointCloud2& msg, const Eigen::Isometry3f& iso)
{
  const double msg_time = rclcpp::Time(msg.header.stamp).seconds();
  const size_t num_points = static_cast<size_t>(msg.height) * msg.width;
  std::vector<types::Point<TConfig>> points(num_points);

  sensor_msgs::PointCloud2ConstIterator<float> it_x(msg, "x");
  sensor_msgs::PointCloud2ConstIterator<float> it_y(msg, "y");
  sensor_msgs::PointCloud2ConstIterator<float> it_z(msg, "z");

  SPHERICAL_FIELDS(DECL_CONST_ITER)
  if constexpr (TConfig::SPHERICAL) {
    if (false SPHERICAL_FIELDS(CHECK_FIELD)) {
      throw std::runtime_error("[OdometryNode]: Missing 'azimuth'/'range'/'elevation' fields in PointCloud2 message");
    }
    SPHERICAL_FIELDS(INIT_CONST_ITER)
  }
  INTENSITY_FIELDS(DECL_CONST_ITER)
  if constexpr (TConfig::INTENSITY) {
    if (false INTENSITY_FIELDS(CHECK_FIELD)) {
      throw std::runtime_error("[OdometryNode]: Missing 'intensity' field in PointCloud2 message");
    }
    INTENSITY_FIELDS(INIT_CONST_ITER)
  }

  RADAR_FIELDS(DECL_CONST_ITER)
  if constexpr (types::HASRADAR<TConfig>) {
    if (false RADAR_FIELDS(CHECK_FIELD)) {
      throw std::runtime_error("[OdometryNode]: Missing radar fields in PointCloud2 message");
    }
    RADAR_FIELDS(INIT_CONST_ITER)
  }

  // Sensor id - field name may be "sensor_id" or "id".
  SENSOR_ID_FIELDS(DECL_CONST_ITER)
  if constexpr (TConfig::SENSOR_ID) {
    if (false SENSOR_ID_FIELDS(CHECK_FIELD)) {
      throw std::runtime_error("[OdometryNode]: Missing 'sensor_id' field in PointCloud2 message");
    }
    SENSOR_ID_FIELDS(INIT_CONST_ITER)
  }

  // Timestamp: optional + variant since the field name and datatype are runtime-determined.
  std::optional<std::variant<sensor_msgs::PointCloud2ConstIterator<uint32_t>,
    sensor_msgs::PointCloud2ConstIterator<float>, sensor_msgs::PointCloud2ConstIterator<double>>>
    msg_ts;
  for (const auto& field : msg.fields) {
    if (field.name == "t" || field.name == "timestamp" || field.name == "time") {
      if (field.datatype == sensor_msgs::msg::PointField::UINT32) {
        msg_ts.emplace(sensor_msgs::PointCloud2ConstIterator<uint32_t>(msg, field.name));
      } else if (field.datatype == sensor_msgs::msg::PointField::FLOAT32) {
        msg_ts.emplace(sensor_msgs::PointCloud2ConstIterator<float>(msg, field.name));
      } else if (field.datatype == sensor_msgs::msg::PointField::FLOAT64) {
        msg_ts.emplace(sensor_msgs::PointCloud2ConstIterator<double>(msg, field.name));
      }
      break;
    }
  }

  // Single pass over points.
  for (size_t i = 0; i < num_points; ++i, ++it_x, ++it_y, ++it_z) {
    auto& pt = points[i];
    pt.pos = iso * Eigen::Vector3f{*it_x, *it_y, *it_z};

    if constexpr (TConfig::SPHERICAL) {
      SPHERICAL_FIELDS(READ_ITER)
    }
    if constexpr (TConfig::INTENSITY) {
      INTENSITY_FIELDS(READ_ITER)
    }
    if constexpr (types::HASRADAR<TConfig>) {
      RADAR_FIELDS(READ_ITER)
    }
    if constexpr (TConfig::SENSOR_ID) {
      SENSOR_ID_FIELDS(READ_ITER)
    }

    if (msg_ts) {
      std::visit(
        [&](auto& it) {
          double ts_s = 0.0;
          auto raw = *it;
          ++it;
          // Different lidar manufacturers use different timestamp formats
          if constexpr (std::is_same_v<typename std::decay_t<decltype(raw)>, uint32_t>) {
            // Timestamp in nanoseconds offset from msg header time (e.g. Ouster format)
            ts_s = static_cast<double>(raw) * 1e-9;
          } else if constexpr (std::is_same_v<typename std::decay_t<decltype(raw)>, float>) {
            // Timestamp in seconds offset from msg header time (e.g. Velodyne format)
            ts_s = static_cast<double>(raw);
          } else {
            // Timestamp in seconds absolute (e.g. TAM format)
            ts_s = static_cast<double>(raw) - msg_time;
          }
          pt.timestamp = static_cast<float>(ts_s);
        },
        *msg_ts);
    }
  }

  return points;
}

/**
 * @brief transform vector of points to sensor_msgs::msg::PointCloud2
 *
 * @param[in] points             - std::vector<types::Point<TConfig>>
 *                                vector of points to convert
 * @param[in] header             - std_msgs::msg::Header
 *                                header for the output message
 * @return sensor_msgs::msg::PointCloud2
 */
template <typename TConfig>
sensor_msgs::msg::PointCloud2 eigen2cloud(
  const std::vector<types::Point<TConfig>>& points, const std_msgs::msg::Header& header)
{
  // Create PointCloud2 message
  sensor_msgs::msg::PointCloud2 cloud_msg;
  sensor_msgs::PointCloud2Modifier modifier(cloud_msg);
  cloud_msg.header = header;
  cloud_msg.fields.clear();
  int offset = 0;
  offset = addPointField(cloud_msg, "x", 1, sensor_msgs::msg::PointField::FLOAT32, offset);
  offset = addPointField(cloud_msg, "y", 1, sensor_msgs::msg::PointField::FLOAT32, offset);
  offset = addPointField(cloud_msg, "z", 1, sensor_msgs::msg::PointField::FLOAT32, offset);
  if constexpr (TConfig::SPHERICAL) SPHERICAL_FIELDS(ADD_FIELD);
  if constexpr (TConfig::INTENSITY) INTENSITY_FIELDS(ADD_FIELD);
  if constexpr (types::HASRADAR<TConfig>) RADAR_FIELDS(ADD_FIELD);
  if constexpr (TConfig::SENSOR_ID) SENSOR_ID_FIELDS(ADD_FIELD);

  offset += sizeOfPointField(sensor_msgs::msg::PointField::FLOAT32);
  // Resize the point cloud accordingly
  cloud_msg.point_step = offset;
  cloud_msg.row_step = cloud_msg.width * cloud_msg.point_step;
  cloud_msg.data.resize(cloud_msg.height * cloud_msg.row_step);
  modifier.resize(points.size());

  // Fill the point cloud data (single pass).
  sensor_msgs::PointCloud2Iterator<float> it_x(cloud_msg, "x");
  sensor_msgs::PointCloud2Iterator<float> it_y(cloud_msg, "y");
  sensor_msgs::PointCloud2Iterator<float> it_z(cloud_msg, "z");

  SPHERICAL_FIELDS(DECL_ITER)
  if constexpr (TConfig::SPHERICAL) {
    SPHERICAL_FIELDS(INIT_ITER)
  }
  INTENSITY_FIELDS(DECL_ITER)
  if constexpr (TConfig::INTENSITY) {
    INTENSITY_FIELDS(INIT_ITER)
  }
  RADAR_FIELDS(DECL_ITER)
  if constexpr (types::HASRADAR<TConfig>) {
    RADAR_FIELDS(INIT_ITER)
  }
  SENSOR_ID_FIELDS(DECL_ITER)
  if constexpr (TConfig::SENSOR_ID) {
    SENSOR_ID_FIELDS(INIT_ITER)
  }

  for (size_t i = 0; i < points.size(); ++i, ++it_x, ++it_y, ++it_z) {
    const Eigen::Vector3f& p = points[i].pos;
    *it_x = p.x();
    *it_y = p.y();
    *it_z = p.z();
    if constexpr (TConfig::SPHERICAL) {
      SPHERICAL_FIELDS(WRITE_ITER)
    }
    if constexpr (TConfig::INTENSITY) {
      INTENSITY_FIELDS(WRITE_ITER)
    }
    if constexpr (types::HASRADAR<TConfig>) {
      RADAR_FIELDS(WRITE_ITER)
    }
    if constexpr (TConfig::SENSOR_ID) {
      SENSOR_ID_FIELDS(WRITE_ITER)
    }
  }
  return cloud_msg;
}

#undef SPHERICAL_FIELDS
#undef INTENSITY_FIELDS
#undef RADAR_FIELDS
#undef SENSOR_ID_FIELDS
#undef DECL_CONST_ITER
#undef INIT_CONST_ITER
#undef CHECK_FIELD
#undef DECL_ITER
#undef INIT_ITER
#undef READ_ITER
#undef WRITE_ITER
#undef ADD_FIELD
}  // namespace utils
}  // namespace tam::core::state
