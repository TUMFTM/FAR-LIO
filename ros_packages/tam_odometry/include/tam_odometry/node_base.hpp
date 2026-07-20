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
#include <atomic>
#include <chrono>
#include <cstdint>
#include <iostream>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

// Pipeline
#include "tam_odometry/utils.hpp"
#include "tam_odometry/node.hpp"
#include "odometry_pipeline/odometry_pipeline.hpp"

// ROS2
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <std_msgs/msg/header.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

// tam libraries
#include "param_management_cpp/param_reference_manager.hpp"
#include "param_management_ros2_integration_cpp/helper_functions.hpp"
#include "ros2_watchdog_cpp/node_monitor.hpp"
#include "ros2_watchdog_cpp/topic_watchdog.hpp"
#include "tsl_ros2_publisher_cpp/tsl_publisher.hpp"
#include "tum_map_msgs/srv/get_point_cloud_map.hpp"
#include "tum_ros_helpers_cpp/pointcloud_file.hpp"
#include "tum_ros_helpers_cpp/qos.hpp"
#include "tum_ros_helpers_cpp/timer.hpp"
#include "tum_ros_helpers_cpp/transform.hpp"

namespace tam::core::state {
template <typename TConfig, typename TNodeConfig, typename TNodeDebug>
class NodeBase : public rclcpp::Node, public OdometryBase<TConfig, TNodeConfig, TNodeDebug>
{
public:
  explicit NodeBase(const rclcpp::NodeOptions& options)
      : rclcpp::Node("NodeBase", options), OdometryBase<TConfig, TNodeConfig, TNodeDebug>(TNodeConfig{}, TNodeDebug{})
  {
    // Call common initialization in derived constructor!
  }

  ~NodeBase()
  {
    // Free the preallocated memory of the OdometryPipeline
    pipeline_->free();
  }

protected:
  /**
   * @brief Regular timer callback
   */
  virtual void timer_callback() = 0;

  /**
   * @brief Initialization sequence for the map
   */
  void init_map()
  {
    // Stop the timer if the map is already initialized.
    if (this->init_status_ != types::InitStatus::WAITING_FOR_MAP) {
      if (this->init_map_timer_) this->init_map_timer_->cancel();
      return;
    }
    if (!this->map_client_) return;
    if (!this->map_client_->service_is_ready()) {
      RCLCPP_WARN(this->get_logger(), "[%s]: Map service not available!", this->get_name());
      return;
    }
    // clang-format off
    RCLCPP_INFO(this->get_logger(), "[%s]: Requesting initial map from %s.", this->get_name(), this->map_client_->get_service_name());  // NOLINT
    // clang-format on
    auto request = std::make_shared<tum_map_msgs::srv::GetPointCloudMap::Request>();
    std_msgs::msg::Header header;
    header.stamp = this->get_clock()->now();
    header.frame_id = this->config_.odom_frame;
    request->header = header;
    request->voxel_size = static_cast<float>(this->pipeline_->get_map_resolution());
    std::function<void(rclcpp::Client<tum_map_msgs::srv::GetPointCloudMap>::SharedFuture)> cb =
      std::bind(&NodeBase::map_callback, this, std::placeholders::_1, true);
    this->map_client_->async_send_request(request, cb);
    // Send a status to the dashboard to indicate that we are waiting for the map
    this->monitor_->set_message("Waiting for map");
    this->monitor_->set_error_lvl("map", tam::types::ErrorLvl::WARN);
    this->monitor_->update();
  }

  /**
   * @brief callback incoming lidar frame
   *
   * @param[in] msg_ptr            - sensor_msgs::msg::PointCloud2::ConstSharedPtr
   *                                 containing lidar frame as sensor_msgs
   * @param[in] diag_msg_ptr       - diagnostic_msgs::msg::DiagnosticArray::ConstSharedPtr
   *                                 containing diagnostic status of the pointcloud source
   */
  void cloud_callback_synced(const sensor_msgs::msg::PointCloud2::ConstSharedPtr msg_ptr,
    const diagnostic_msgs::msg::DiagnosticArray::ConstSharedPtr diag_msg_ptr)
  {
    // Set status
    this->pipeline_->set_input_status(utils::convert_diag_status(diag_msg_ptr));

    // Call regular cloud callback
    this->cloud_callback(msg_ptr);
  }

  /**
   * @brief callback incoming lidar frame
   *
   * @param[in] msg_ptr            - sensor_msgs::msg::PointCloud2::ConstSharedPtr
   *                                 containing lidar frame as sensor_msgs
   */
  virtual void cloud_callback(const sensor_msgs::msg::PointCloud2::ConstSharedPtr msg_ptr) = 0;

  /**
   * @brief send diagnostics in case of timeout of incoming ros2 topic
   *
   * @param[in] trigger            - bool
   *                                whether the timeout was triggered
   * @param[in] timeout            - std::chrono::milliseconds
   *                                containing allowed timeout time for topic
   */
  void timeout_callback_pointcloud(bool trigger, std::chrono::milliseconds timeout)
  {
    if (trigger && this->init_status_ == types::InitStatus::READY) {
      RCLCPP_ERROR(this->get_logger(), "[%s]: No pointcloud received for %ldms!", this->get_name(), timeout.count());
      this->monitor_->set_message("Pointcloud Timeout");
      this->monitor_->set_error_lvl("pointcloud_timeout", tam::types::ErrorLvl::ERROR);
      // Publish the monitor status as this is the timeout callback corresponding to
      // the callback of the pointcloud
      this->monitor_->update();
    } else {
      // Reset diagnostics for input cloud
      this->monitor_->set_error_lvl("pointcloud_timeout", tam::types::ErrorLvl::OK);
    }
  }

  /**
   * @brief Callback to update the map asynchronously
   * @param [in] msg            - std_msgs::msg::Header::SharedPtr
   *                              timestamp to update the map for
   */
  void update_ready_callback(const std_msgs::msg::Header::SharedPtr msg)
  {
    if (this->init_status_ != types::InitStatus::READY) {
      return;
    }
    // Check if an update is already in progress
    bool expected = false;
    // clang-format off
    if (!this->update_in_progress_.compare_exchange_strong(expected, true)) {
      RCLCPP_WARN(this->get_logger(), "[%s]: Received map update while still in progress - skipping", this->get_name());  // NOLINT
      return;
    }
    this->debug_.map_update = true;
    // clang-format on
    // Request map update
    // Check if we can get the map from the service
    if (this->update_map_client_ && this->update_map_client_->service_is_ready()) {
      // clang-format off
      RCLCPP_INFO(this->get_logger(), "[%s]: Requesting map update from %s.", this->get_name(), this->update_map_client_->get_service_name());  // NOLINT
      auto request = std::make_shared<tum_map_msgs::srv::GetPointCloudMap::Request>();
      request->header = *msg;
      request->voxel_size = static_cast<float>(this->pipeline_->get_map_resolution());
      std::function<void(rclcpp::Client<tum_map_msgs::srv::GetPointCloudMap>::SharedFuture)> cb =
        std::bind(&NodeBase::map_callback, this, std::placeholders::_1, false);
      this->update_map_client_->async_send_request(request, cb);
      // clang-format on
    }
  }

  /**
   * @brief Map callback for the service response
   * @param [in] future           -
   * rclcpp::Client<tum_map_msgs::srv::GetPointCloudMap>::SharedFuture containing the map response
   * @param [in] initial_map      - bool indicating if this is the initial map load or an async
   * update
   */
  void map_callback(
    const rclcpp::Client<tum_map_msgs::srv::GetPointCloudMap>::SharedFuture future, const bool initial_map)
  {
    auto response = future.get();
    const std::string service_name =
      initial_map ? this->map_client_.get()->get_service_name() : this->update_map_client_.get()->get_service_name();
    if (!response->success) {
      // clang-format off
      RCLCPP_ERROR(this->get_logger(), "[%s]: Map service %s returned failure - skipping", this->get_name(), service_name.c_str());  // NOLINT
      // clang-format on
      if (!initial_map) this->update_in_progress_.store(false);
      return;
    }

    // Helper lambda to load the map and print the time taken for loading
    auto load = [&](const tum_map_msgs::srv::GetPointCloudMap::Response::SharedPtr response,
                  const std::string& service_name) {
      const auto start = std::chrono::high_resolution_clock::now();
      const sensor_msgs::msg::PointCloud2 cloud = tam::ros::read_pointcloud_file(response->map.pointcloud);
      tam::ros::remove_pointcloud_file(response->map.pointcloud);
      // clang-format off
      const auto time_ms = std::chrono::duration<double, std::milli>(std::chrono::high_resolution_clock::now() - start).count();  // NOLINT
      RCLCPP_INFO(this->get_logger(), "[%s]: Received map with %u points from %s (%.2f ms)", this->get_name(), cloud.width * cloud.height, service_name.c_str(), time_ms);  // NOLINT
      // clang-format on
      return utils::cloud2eigen<types::Point_XYZ>(cloud, Eigen::Isometry3f::Identity());
    };

    if (initial_map) {
      // Just add the points to the pipeline if its the initial load
      auto points = load(response, service_name);
      this->pipeline_->add_points(points);
      // Publish the map if in debug mode
      if (this->pipeline_->get_config().debug_mode && this->map_publisher_) {
        std_msgs::msg::Header header;
        header.stamp = this->get_clock()->now();
        header.frame_id = this->config_.odom_frame;
        this->map_publisher_->publish(utils::eigen2cloud<TConfig>(this->pipeline_->get_map(), header));
      }
      // Set the status and cancel the init-map retry timer
      this->init_status_ = types::InitStatus::WAITING_FOR_EKF;
      this->monitor_->set_error_lvl("map", tam::types::ErrorLvl::OK);
      if (this->init_map_timer_) this->init_map_timer_->cancel();
    } else {
      // if its an async update, start a new thread to load the map and request the pipeline to
      // switch the map atomically
      std::thread([this, response, service_name, load]() {
        auto points = load(response, service_name);
        {
          std::lock_guard<std::mutex> lock(this->pipeline_->get_map_mutex());
          this->pipeline_->add_points(points, TConfig::MAX_POINTS_PER_VOXEL, 1, TConfig::NUM_NEIGHBORS, false);
          this->pipeline_->request_map_switch();
        }
        // clang-format off
          RCLCPP_INFO(this->get_logger(), "[%s]: Asynchronous map update applied (%lu points)", this->get_name(), points.size());  // NOLINT
        // clang-format on
        this->update_in_progress_.store(false);
        // Write the debug signal
        // CAUTION: Set from the worker thread, but should be fine as only read from the main thread
        // and written by at most one worker thread at the same time
        this->debug_.map_update = false;
      }).detach();
    }
  }

  /**
   * @brief Set undistortion poses for the pipeline by sampling tf
   *
   * Aligns the frame stamp to the 10ms grid and samples the last 200ms.
   * Only samples newer than the last fed stamp are queried to avoid looking up the
   * same poses twice on overlapping frames.
   *
   * @param[in] stamp           Timestamp of the incoming frame
   */
  void set_undistortion_poses(const std::uint64_t& stamp)
  {
    if (!this->pipeline_->get_config().undistort) {
      return;
    }
    constexpr std::uint64_t UNDISTORTION_INTERVAL_NS = 10'000'000ULL;  // 10ms
    const std::uint64_t frame_ns = stamp;
    // Round down to the 10ms grid so samples align across frames
    const std::uint64_t latest_interval_ns = frame_ns - frame_ns % UNDISTORTION_INTERVAL_NS;
    std::string tf_error;
    std::uint64_t newest_set_stamp = this->distortion_pose_stamp_;
    bool gap = false;
    for (int i = 19; i >= 0; --i) {
      // clang-format off
      const std::uint64_t sample_ns = latest_interval_ns - static_cast<std::uint64_t>(i) * UNDISTORTION_INTERVAL_NS;  // NOLINT
      // Skip samples already set in a previous callback
      if (sample_ns <= this->distortion_pose_stamp_) {
        continue;
      }
      const rclcpp::Time sample_time(sample_ns);
      if (this->tf_buffer_->canTransform(this->config_.odom_frame, this->config_.child_frame, sample_time, tf2::durationFromSec(0.0), &tf_error)) {  // NOLINT
        this->pipeline_->set_pose_undistortion(utils::transform2pose(this->tf_buffer_->lookupTransform(this->config_.odom_frame, this->config_.child_frame, sample_time)));  // NOLINT
        // Only advance while the fed samples are contiguous (no earlier lookup has failed)
        if (!gap) {
          newest_set_stamp = sample_ns;
        }
      } else {
        gap = true;
      }
      // clang-format on
    }
    this->distortion_pose_stamp_ = newest_set_stamp;
  }

protected:
  /**
   * @brief Initialize common parts of the node
   */
  void init_node_common()
  {
    // Construct from param manager and logger
    this->pmg_ = std::make_shared<tam::pmg::ParamReferenceManager>();
    this->logger_ = std::make_shared<tam::tsl::ReferenceLogger>();

    // Declare node level params and debug values
    this->set_config(this->pmg_.get());
    this->set_logging(this->logger_.get());

    // Inizialize odometry pipeline to declare all params
    this->pipeline_ = OdometryPipeline<TConfig>::from_config(this->pmg_.get(), this->logger_.get());

    // Connect your param manager to the callback
    // Important: don't forget to store your callback handle here, otherwise it will get
    // deallocated
    auto callback_handle = tam::pmg::connect_param_manager_to_ros_cb(this, pmg_);
    // Declare all paramters from the param manager
    tam::pmg::declare_ros_params_from_param_manager(this, pmg_.get());

    // Now that the param overrides are active, init necesarry modules
    // Initialize pipeline
    this->pipeline_->init();

    // Initialize monitoring of watchdog
    this->monitor_ = std::make_unique<tam::core::NodeMonitor>(this, this->config_.output_odom);
    this->topic_watchdog_ = std::make_unique<tam::core::TopicWatchdog>(this);

    // Initialize QoS
    rclcpp::QoS qos_lidar = tam::ros::get_qos(tam::ros::TopicType::PERCEPTION_SENSOR_DATA);
    rclcpp::QoS qos_odom = tam::ros::get_qos(tam::ros::TopicType::DEFAULT);

    // Initialize tf listener
    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(get_clock());
    tf_listener_ = std::make_unique<tf2_ros::TransformListener>(*tf_buffer_);

    // Initialize logger
    tsl_publisher_ = std::make_unique<tam::tsl::TSLPublisher>(this, this->logger_);

    // Load static map either from a local .pcd file or from the map loader service
    if (!this->pipeline_->get_config().update_map) {
      const std::string& input_map = this->config_.input_map;
      if (input_map.ends_with(".pcd")) {
        this->pipeline_->add_points(utils::load_static_map<tam::core::state::types::Point_XYZ>(input_map));
        this->init_status_ = types::InitStatus::WAITING_FOR_EKF;
      } else {
        // Create client to request the map from service (within a separate timer)
        this->map_client_ = this->create_client<tum_map_msgs::srv::GetPointCloudMap>(input_map);
        this->init_map_timer_ =
          tam::create_timer(this, std::chrono::milliseconds(5000), std::bind(&NodeBase::init_map, this));
      }
    } else {
      // If we don't need a map at initialization, we can directly wait for the EKF handshake
      this->init_status_ = types::InitStatus::WAITING_FOR_EKF;
    }

    // Intialize subscribers and callbacks
    // Subscribe tof input pointcloud with optional diagnostics
    if (this->pipeline_->get_diagnostic_config().check_input_status) {
      // Both subscriptions need to have the same QoS settings
      auto sub_pc = topic_watchdog_->add_synced_subscription<sensor_msgs::msg::PointCloud2>(
        this->config_.input_pointcloud, qos_lidar);
      auto sub_pc_diag = topic_watchdog_->add_synced_subscription<diagnostic_msgs::msg::DiagnosticArray>(
        this->config_.input_pointcloud_status, qos_lidar);
      topic_watchdog_->register_synced_callback(sub_pc, sub_pc_diag,
        std::bind(&NodeBase::cloud_callback_synced, this, std::placeholders::_1, std::placeholders::_2),
        std::bind(&NodeBase::timeout_callback_pointcloud, this, std::placeholders::_1, std::placeholders::_2),
        std::chrono::milliseconds(static_cast<int64_t>(500.0)));
    } else {
      // just subscribe to the pointcloud topic without diagnostics using the downstream
      // callback
      topic_watchdog_->add_subscription<sensor_msgs::msg::PointCloud2>(this->config_.input_pointcloud, qos_lidar,
        std::bind(&NodeBase::cloud_callback, this, std::placeholders::_1),
        std::bind(&NodeBase::timeout_callback_pointcloud, this, std::placeholders::_1, std::placeholders::_2),
        std::chrono::milliseconds(static_cast<int64_t>(500.0)));
    }

    // Map Update service and subscriber for update trigger
    if (!this->config_.update_map_srv.empty()) {
      this->update_map_client_ = this->create_client<tum_map_msgs::srv::GetPointCloudMap>(this->config_.update_map_srv);
      const std::string update_ready_topic = this->config_.update_map_srv + "_ready";
      this->update_ready_sub_ = this->create_subscription<std_msgs::msg::Header>(
        update_ready_topic, qos_odom, std::bind(&NodeBase::update_ready_callback, this, std::placeholders::_1));
    }

    // Initialize publishers
    this->odom_publisher_ = this->create_publisher<nav_msgs::msg::Odometry>(this->config_.output_odom, qos_odom);

    // Debug mode publishers
    if (this->pipeline_->get_config().debug_mode) {
      this->frame_publisher_ =
        this->create_publisher<sensor_msgs::msg::PointCloud2>("/core/state/frame_registration", qos_lidar);
      if (!this->pipeline_->get_config().update_map) {
        // If static map is used, only publish once now transient local
        this->map_publisher_ =
          this->create_publisher<sensor_msgs::msg::PointCloud2>("/core/state/map", tam::ros::get_qos(tam::ros::EVENT));
        if (!this->map_client_) {
          // Map already available from file, publish it now
          std_msgs::msg::Header header;
          header.stamp = this->get_clock()->now();
          header.frame_id = this->config_.odom_frame;
          this->map_publisher_->publish(utils::eigen2cloud<TConfig>(this->pipeline_->get_map(), header));
        }
      } else {
        // If map is updated, publish with QoS
        this->map_publisher_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/core/state/map", qos_lidar);
      }
    }
    // Set static transform for pointcloud
    this->pc_transform_ = tam::ros::transform2eigen(
      tam::ros::wait_static_transform(this, tf_buffer_.get(), this->config_.child_frame, this->config_.cloud_frame));

    // Initialize timer
    this->timer_ = tam::create_timer(this, 100ms, std::bind(&NodeBase::timer_callback, this));
  }

  /**
   * @brief Declare common configuration parameters for the node
   */
  void set_config_common(tam::pmg::ParamReferenceManager* pmg)
  {
    // clang-format off
    pmg->declare_parameter("node.odom_frame", &this->config_.odom_frame, "", tam::pmg::ParameterType::STRING, "Frame id of the output odometry");  // NOLINT
    pmg->declare_parameter("node.child_frame", &this->config_.child_frame, "", tam::pmg::ParameterType::STRING, "Child frame id of the output odometry (vehicle frame)");  // NOLINT
    pmg->declare_parameter("node.cloud_frame", &this->config_.cloud_frame, "", tam::pmg::ParameterType::STRING, "Frame id of the input pointcloud (sensor frame)");  // NOLINT
    pmg->declare_parameter("node.input_pointcloud", &this->config_.input_pointcloud, "/vehicle/sensor/lidar", tam::pmg::ParameterType::STRING, "Topic of the input pointcloud");  // NOLINT
    pmg->declare_parameter("node.input_pointcloud_status", &this->config_.input_pointcloud_status, "/vehicle/sensor/lidar_status", tam::pmg::ParameterType::STRING, "Topic of the input pointcloud status");  // NOLINT
    pmg->declare_parameter("node.output_odom", &this->config_.output_odom, "/core/state/lidar_odometry", tam::pmg::ParameterType::STRING, "Topic for the odometry output");  // NOLINT
    pmg->declare_parameter("node.input_map", &this->config_.input_map, "/core/map/get_pointcloud_map", tam::pmg::ParameterType::STRING, "Path to a .pcd file to load or name of the GetPointCloudMap service");  // NOLINT
    pmg->declare_parameter("node.update_map_srv", &this->config_.update_map_srv, "", tam::pmg::ParameterType::STRING, "Name of the GetPointCloudMap service for asynchronous map updates. The trigger topic is derived by appending '_ready'. Leave empty to disable.");  // NOLINT
                                             // clang-format on
  }

  /**
   * @brief Declare common logging parameters for the node
   */
  void set_logging_common(tam::tsl::ReferenceLogger* logger) const
  {
    logger->log("node/map_update", &this->debug_.map_update);
    logger->log("node/cloud_callback_time", &this->debug_.callback_time);
  }

protected:
  // Initialization
  types::InitStatus init_status_ = types::InitStatus::WAITING_FOR_MAP;
  // param manager
  std::shared_ptr<tam::pmg::ParamReferenceManager> pmg_;
  // logger
  std::shared_ptr<tam::tsl::ReferenceLogger> logger_;
  // Node monitor
  tam::core::NodeMonitor::UniquePtr monitor_;
  tam::core::TopicWatchdog::UniquePtr topic_watchdog_;
  // odometry pipeline
  std::unique_ptr<OdometryPipeline<TConfig>> pipeline_;
  // Static point cloud transform
  Eigen::Isometry3f pc_transform_{};
  // Stamp (ns) of the most recent deskewing pose fed from tf, to avoid double-querying poses
  std::uint64_t distortion_pose_stamp_{0};
  // Timer
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::TimerBase::SharedPtr init_map_timer_{};
  // tf_listener and buffer.
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::unique_ptr<tf2_ros::TransformListener> tf_listener_;
  // Odometry output
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_publisher_;
  // Debug publisher
  tam::tsl::TSLPublisher::UniquePtr tsl_publisher_{};
  // Pointcloud publishers - only used in debug_mode
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr frame_publisher_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr map_publisher_;
  rclcpp::Publisher<geometry_msgs::msg::PolygonStamped>::SharedPtr ellipsis_;  // Not implemented
  // Client to request the map from the tam_map_loader service
  rclcpp::Client<tum_map_msgs::srv::GetPointCloudMap>::SharedPtr map_client_{};
  // Async map update: client, trigger subscription, and worker-thread guard
  rclcpp::Client<tum_map_msgs::srv::GetPointCloudMap>::SharedPtr update_map_client_{};
  rclcpp::Subscription<std_msgs::msg::Header>::SharedPtr update_ready_sub_{};
  std::atomic<bool> update_in_progress_{false};
};
}  // namespace tam::core::state
