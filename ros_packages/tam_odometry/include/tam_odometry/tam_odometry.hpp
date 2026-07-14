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
// Common libraries
#include <chrono>
#include <iostream>
#include <memory>
#include <string>
#include <vector>

// Node base
#include "tam_odometry/node_base.hpp"

// ROS2
#include <tf2_ros/transform_broadcaster.h>
//
namespace tam::core::state
{
template <typename TConfig>
class OdometryNode : public NodeBase<TConfig, types::OdometryNodeConfig, types::OdometryNodeDebug>
{
public:
  explicit OdometryNode(const rclcpp::NodeOptions & options)
  : NodeBase<TConfig, types::OdometryNodeConfig, types::OdometryNodeDebug>(options)
  {
    // Call common initialization here to be able to call set_config/set_logging,
    // which are pure virtual in the base class
    this->init_node_common();
    // Initialize node and pipeline if not using external guess
    if constexpr (TConfig::MODEL != types::ModelType::EXTERNALGUESS) {
      // Set member variable for initialization to true as waiting for odometry is not necessary
      this->init_status_ = types::InitStatus::READY;
      // Initialize model
      this->pipeline_->init_model_config();
      // Initialize dynamic transform broadcaster
      this->tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
    }

    // Finish
    this->monitor_->initialization_finished();
    std::cout << "tam_odometry node initialized!" << std::endl;
  }
  ~OdometryNode() override = default;

private:
  void timer_callback() override
  {
    // Check for timeouts
    this->topic_watchdog_->check_timeouts();
    // Check initial guess status
    if (this->init_status_ == types::InitStatus::WAITING_FOR_EKF && this->config_.wait_tf) {
      std::string tf_error{};
      if (
        this->tf_buffer_->canTransform(
          this->config_.odom_frame, this->config_.child_frame, tf2::TimePointZero,
          tf2::durationFromSec(0.0), &tf_error)) {
        const types::PoseStamped pose = utils::transform2pose(this->tf_buffer_->lookupTransform(
          this->config_.odom_frame, this->config_.child_frame, tf2::TimePointZero));  // NOLINT
        this->pipeline_->set_pose_model(pose, true);
        // Finish initialization
        RCLCPP_INFO(this->get_logger(), "[%s]: Handshake successful!", this->get_name());
        this->monitor_->set_message("Initialized");
        this->monitor_->set_error_lvl("handshake", tam::types::ErrorLvl::OK);
        this->init_status_ = types::InitStatus::READY;
      } else {
        this->monitor_->set_message("Waiting for handshake with EKF");
        this->monitor_->set_error_lvl("handshake", tam::types::ErrorLvl::WARN);
        RCLCPP_WARN(this->get_logger(), "[%s]: Waiting for handshake with EKF", this->get_name());
      }
      // Send a status to the dashboard to indicate the initialization status
      this->monitor_->update();
    }
  }
  /**
   * @brief callback incoming lidar frame
   *
   * @param[in] msg_ptr            - sensor_msgs::msg::PointCloud2::ConstSharedPtr
   *                                 containing lidar frame as sensor_msgs
   */
  void cloud_callback(const sensor_msgs::msg::PointCloud2::ConstSharedPtr msg_ptr) override
  {
    // Return if not initialized
    if (this->init_status_ != types::InitStatus::READY && this->config_.wait_tf) {
      return;
    }

    // Start timer
    auto start = std::chrono::high_resolution_clock::now();

    // clang-format off
    // Convert message to points
    if (msg_ptr->header.frame_id != this->config_.cloud_frame) {
      RCLCPP_ERROR(this->get_logger(), "[%s]: Frame id of pointcloud differs from specified one!", this->get_name());  // NOLINT
    }
    std::vector<types::Point<TConfig>> frame = utils::cloud2eigen<TConfig>(*msg_ptr, this->pc_transform_); // NOLINT
    // Fetch initial guess and set it to pipeline if configured
    if constexpr (TConfig::MODEL == types::ModelType::EXTERNALGUESS) {
      types::PoseStamped init_guess{};
      bool valid_guess{true};
      std::string tf_error;
      // Try to fetch initial guess from tf
        if (this->tf_buffer_->canTransform(this->config_.odom_frame, this->config_.child_frame, msg_ptr->header.stamp, tf2::durationFromSec(0.0), &tf_error)) {  // NOLINT
          init_guess = utils::transform2pose(this->tf_buffer_->lookupTransform(this->config_.odom_frame, this->config_.child_frame, msg_ptr->header.stamp));  // NOLINT
        } else if (this->tf_buffer_->canTransform(this->config_.odom_frame, this->config_.child_frame, tf2::TimePointZero, tf2::durationFromSec(0.0), &tf_error)) {  // NOLINT
          init_guess = utils::transform2pose(this->tf_buffer_->lookupTransform(this->config_.odom_frame, this->config_.child_frame, tf2::TimePointZero));  // NOLINT
        } else {
          RCLCPP_WARN(this->get_logger(), "[%s]: Could not lookup initial guess available from tf!", this->get_name());  // NOLINT
          const auto odom = this->pipeline_->get_odometry();
          init_guess.pose = odom.pose.pose;
          init_guess.stamp = utils::stamp2ns(msg_ptr->header.stamp);
          valid_guess = false;
        }
      this->pipeline_->set_pose_model(init_guess, valid_guess);
    }
    // clang-format on
    // Set deskewing poses to the pipeline by sampling tf around the frame stamp
    this->set_undistortion_poses(utils::stamp2ns(msg_ptr->header.stamp));
    // Register frame
    types::Odometry odom = this->pipeline_->register_frame(
      frame, utils::stamp2ns(msg_ptr->header.stamp));

    // Compute elapsed time
    // clang-format off
    double time = std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::high_resolution_clock::now() - start).count() * 1.0e-3;  // NOLINT
    // clang-format on
    this->debug_.callback_time = time;
    // Initialize if the pose is valid
    if (odom.status.level == types::DiagnosticLevel::OK) {
      this->monitor_->set_error_lvl("handshake", tam::types::ErrorLvl::OK);
      this->init_status_ = types::InitStatus::READY;
    }
    // Publish diagnostics
    utils::set_monitor(odom, this->monitor_.get());
    const nav_msgs::msg::Odometry odom_msg =
      utils::odom2msg(odom, this->config_.odom_frame, this->config_.child_frame);
    // Publish status and odometry with same stamp
    this->monitor_->update(odom_msg.header);
    this->odom_publisher_->publish(odom_msg);
    // Publish tf if no external guess
    if constexpr (TConfig::MODEL != types::ModelType::EXTERNALGUESS) {
      geometry_msgs::msg::TransformStamped transform_msg;
      transform_msg.header.stamp = msg_ptr->header.stamp;
      transform_msg.header.frame_id = this->config_.odom_frame;
      transform_msg.child_frame_id = this->config_.child_frame;
      transform_msg.transform = utils::sophus2transform(odom.pose.pose);
      tf_broadcaster_->sendTransform(transform_msg);
    }
    // Publish debug signals
    this->tsl_publisher_->trigger();
    // Publish frame if debug mode is enabled
    if (this->pipeline_->get_config().debug_mode) {
      // Publish frame
      std_msgs::msg::Header cloud_header = msg_ptr->header;
      cloud_header.frame_id = this->config_.child_frame;
      this->frame_publisher_->publish(
        utils::eigen2cloud<TConfig>(this->pipeline_->get_frame(), cloud_header));
      // Publish map
      if (this->pipeline_->get_config().update_map) {
        cloud_header.frame_id = this->config_.odom_frame;
        this->map_publisher_->publish(
          utils::eigen2cloud<TConfig>(this->pipeline_->get_map(), cloud_header));
      }
    }
  }

private:
  /**
   * @brief Declare configuration parameters for the node
   */
  void set_config(tam::pmg::ParamReferenceManager * pmg) override
  {
    // Call common config
    this->set_config_common(pmg);
    // clang-format off
    pmg->declare_parameter("node.wait_tf", &this->config_.wait_tf, true, tam::pmg::ParameterType::BOOL, "Wait for the initial pose from tf (EKF) before starting");  // NOLINT
    // clang-format on
  }
  /**
   * @brief Declare logging parameters for the node
   */
  void set_logging(tam::tsl::ReferenceLogger * logger) const override
  {
    // Call common logging
    this->set_logging_common(logger);
    logger->log("node/cloud_callback_time", &this->debug_.callback_time);
  }

private:
  // tf broadcaster
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_{nullptr};
};
}  // namespace tam::core::state
