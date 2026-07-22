/*
 * Copyright 2023 Marcel Weinmann, Maximilian Leitenstern
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

#include <boost/algorithm/string.hpp>
#include <boost/range/iterator_range.hpp>
#include <cmath>
#include <memory>
#include <stdexcept>
#include <string>
#include <variant>
#include <vector>

// ROS
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>

#include <rclcpp/rclcpp.hpp>

#include "tf2/exceptions.h"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_broadcaster.h"
#include "tf2_ros/transform_listener.h"

// type definitions
#include "tam_state_estimation_node/helper.hpp"
#include "tum_helpers_cpp/containers.hpp"
#include "types.hpp"

// 3d to 2d helper functions
#include "tum_helpers_cpp/rotations.hpp"

// type conversions
#include "tum_type_conversions_ros_cpp/orientation.hpp"
#include "tum_type_conversions_ros_cpp/tum_type_conversions.hpp"

// messages
#include "autoware_auto_vehicle_msgs/msg/steering_report.hpp"
#include "diagnostic_msgs/msg/diagnostic_array.hpp"
#include "geometry_msgs/msg/accel_with_covariance_stamped.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "tum_msgs/msg/tum_float64_per_wheel.hpp"
#include "tum_msgs/msg/tum_float64_per_wheel_stamped.hpp"

// state estimation
#include "state_estimation_base/state_estimation_base.hpp"
#include "tam_state_estimation/state_estimation.hpp"

// State Estimation constants / template input
#include "state_estimation_constants/EKF_2D.hpp"
#include "state_estimation_constants/EKF_3D.hpp"

// delay compensation
#include "tum_helpers_cpp/delay_compensation.hpp"

// param manager
#include "param_management_cpp/param_manager_composer.hpp"
#include "param_management_cpp/param_value_manager.hpp"
#include "param_management_ros2_integration_cpp/helper_functions.hpp"

// tsl logger
#include "tsl_logger_cpp/value_logger.hpp"
#include "tsl_ros2_publisher_cpp/tsl_publisher.hpp"

// topic watchdog
#include "ros2_watchdog_cpp/topic_watchdog.hpp"

// node monitor
#include "ros2_watchdog_cpp/node_monitor.hpp"

// helper
#include "tum_ros_helpers_cpp/qos.hpp"
#include "tum_ros_helpers_cpp/timer.hpp"

using namespace std::chrono_literals;
using std::placeholders::_1;
using std::placeholders::_2;

template <class TConfig, tam::core::state::VehicleModel TModel>
class stateEstimationNode : public rclcpp::Node
{
public:
  explicit stateEstimationNode(
    std::unique_ptr<tam::core::state::StateEstimationBase>&& state_estimation, const rclcpp::NodeOptions& options)
      : Node("StateEstimation", "/core/state", options), state_estimation_(std::move(state_estimation))
  {
    // Initialize the transform broadcaster
    // has to be done before initializing the param manager
    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(this);

    // initialize TransformListener
    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    // Initialize the ParamManagerBase
    param_manager_composer_ = std::make_shared<tam::pmg::ParamManagerComposer>(
      std::vector<tam::pmg::MgmtInterface::SharedPtr>{param_manager_, state_estimation_->get_param_handler()});
    tam::pmg::MgmtInterface* param_manager_raw = param_manager_composer_.get();
    this->declare_and_update_parameters();

    // Parse the measurement / input sensor configuration from the ROS parameter overrides
    std::vector<std::pair<tam::types::state::measurements::identifier, std::string>> identifier_names{};
    tam::core::state::parse_sensor_configs<TConfig>(
      this, param_manager_raw, measurement_channels_, input_channels_, identifier_names);

    callback_handle_ = tam::pmg::connect_param_manager_to_ros_cb(this, param_manager_composer_);
    tam::pmg::declare_ros_params_from_param_manager(this, param_manager_raw);
    this->declare_and_update_parameters();

    // asemble Measurement Covariance Noise Matrix (R) and outlier bounds vector for Kalman filter
    std::vector<double> R_int = std::vector<double>(TConfig::MEASUREMENT_VECTOR_SIZE, 1.0e9);
    std::vector<double> outlier_bounds = std::vector<double>(TConfig::MEASUREMENT_VECTOR_SIZE, 1.0e9);
    // clang-format off
    for (const auto& measurement : measurement_channels_) {
      if (measurement.config.position_m.active) {
        const uint8_t begin_offset = measurement.config.position_m.num * TConfig::POS_MEASUREMENT_VECTOR_SIZE;
        if (begin_offset + measurement.config.position_m.R_init.size() > R_int.size())
          throw std::invalid_argument("[StateEstimation]: too many/oversized 'position_m' fusion configs (exceeds MEASUREMENT_VECTOR_SIZE) for " + measurement.config.name);  // NOLINT
        std::copy(measurement.config.position_m.R_init.begin(), measurement.config.position_m.R_init.end(), R_int.begin() + begin_offset);  // NOLINT
        std::copy(measurement.config.position_m.outlier_bounds.begin(), measurement.config.position_m.outlier_bounds.end(), outlier_bounds.begin() + begin_offset);  // NOLINT
        identifier_names.emplace_back(tam::types::state::measurements::identifier{tam::types::state::measurements::POS, measurement.config.position_m.num}, measurement.config.name);  // NOLINT
      }
      if (measurement.config.orientation_rad.active) {
        const uint8_t begin_offset = TConfig::MEASUREMENT_VECTOR_OFFSET_ORIENTATION + measurement.config.orientation_rad.num * TConfig::ORIENTATION_MEASUREMENT_VECTOR_SIZE;  // NOLINT
        if (begin_offset + measurement.config.orientation_rad.R_init.size() > R_int.size())
          throw std::invalid_argument("[StateEstimation]: too many/oversized 'orientation_rad' fusion configs (exceeds MEASUREMENT_VECTOR_SIZE) for " + measurement.config.name);  // NOLINT
        std::copy(measurement.config.orientation_rad.R_init.begin(), measurement.config.orientation_rad.R_init.end(), R_int.begin() + begin_offset);  // NOLINT
        std::copy(measurement.config.orientation_rad.outlier_bounds.begin(), measurement.config.orientation_rad.outlier_bounds.end(), outlier_bounds.begin() + begin_offset);  // NOLINT
        identifier_names.emplace_back(tam::types::state::measurements::identifier{tam::types::state::measurements::ORIENTATION, measurement.config.orientation_rad.num}, measurement.config.name);  // NOLINT
      }
      if (measurement.config.linear_velocity_mps.active) {
        const uint8_t begin_offset = TConfig::MEASUREMENT_VECTOR_OFFSET_VEL + measurement.config.linear_velocity_mps.num * TConfig::VEL_MEASUREMENT_VECTOR_SIZE;  // NOLINT
        if (begin_offset + measurement.config.linear_velocity_mps.R_init.size() > R_int.size())
          throw std::invalid_argument("[StateEstimation]: too many/oversized 'linear_velocity_mps' fusion configs (exceeds MEASUREMENT_VECTOR_SIZE) for " + measurement.config.name);  // NOLINT
        std::copy(measurement.config.linear_velocity_mps.R_init.begin(), measurement.config.linear_velocity_mps.R_init.end(), R_int.begin() + begin_offset);  // NOLINT
        std::copy(measurement.config.linear_velocity_mps.outlier_bounds.begin(), measurement.config.linear_velocity_mps.outlier_bounds.end(), outlier_bounds.begin() + begin_offset);  // NOLINT
        identifier_names.emplace_back(tam::types::state::measurements::identifier{tam::types::state::measurements::VEL, measurement.config.linear_velocity_mps.num}, measurement.config.name);  // NOLINT
      }
    }
    // clang-format on

    // clang-format off
    if (param_manager_raw->get_value("enable.reference_angle_measurement").as_bool()) {
      const uint8_t begin_offset = TConfig::MEASUREMENT_VECTOR_OFFSET_ORIENTATION + (TConfig::NUM_ORIENTATION_MEASUREMENT - 1) * TConfig::ORIENTATION_MEASUREMENT_VECTOR_SIZE; // NOLINT
      const std::vector<double> reference_angles_R_int = param_manager_raw->get_value("reference_angles.orientation.R_init").as_double_array(); // NOLINT
      const std::vector<double> reference_angles_outlier_bounds = param_manager_raw->get_value("reference_angles.orientation.outlier_bounds").as_double_array(); // NOLINT
      std::copy(reference_angles_R_int.begin(), reference_angles_R_int.end(), R_int.begin() + begin_offset); // NOLINT
      std::copy(reference_angles_outlier_bounds.begin(), reference_angles_outlier_bounds.end(), outlier_bounds.begin() + begin_offset); // NOLINT
    }
    // clang-format on

    // clang-format off
    if (param_manager_raw->get_value("enable.external_orientation_measurement").as_bool()) {
      const uint8_t begin_offset = TConfig::MEASUREMENT_VECTOR_OFFSET_ORIENTATION + (TConfig::NUM_ORIENTATION_MEASUREMENT - 2) * TConfig::ORIENTATION_MEASUREMENT_VECTOR_SIZE; // NOLINT
      const std::vector<double> external_orientation_R_int = param_manager_raw->get_value("external_orientation.orientation.R_init").as_double_array(); // NOLINT
      const std::vector<double> external_orientation_outlier_bounds = param_manager_raw->get_value("external_orientation.orientation.outlier_bounds").as_double_array(); // NOLINT
      std::copy(external_orientation_R_int.begin(), external_orientation_R_int.end(), R_int.begin() + begin_offset); // NOLINT
      std::copy(external_orientation_outlier_bounds.begin(), external_orientation_outlier_bounds.end(), outlier_bounds.begin() + begin_offset); // NOLINT
    }
    // clang-format on

    // clang-format off
    if constexpr (TModel != tam::core::state::VehicleModel::Kinematic) {
      const uint8_t begin_offset = TConfig::MEASUREMENT_VECTOR_OFFSET_VEL + (TConfig::NUM_VEL_MEASUREMENT - 1) * TConfig::VEL_MEASUREMENT_VECTOR_SIZE; // NOLINT
      const std::vector<double> vehicle_model_R_int = param_manager_raw->get_value("vehicle_model.linear_velocity_mps.R_init").as_double_array(); // NOLINT
      const std::vector<double> vehicle_model_outlier_bounds = param_manager_raw->get_value("vehicle_model.linear_velocity_mps.outlier_bounds").as_double_array(); // NOLINT
      std::copy(vehicle_model_R_int.begin(), vehicle_model_R_int.end(), R_int.begin() + begin_offset); // NOLINT
      std::copy(vehicle_model_outlier_bounds.begin(), vehicle_model_outlier_bounds.end(), outlier_bounds.begin() + begin_offset); // NOLINT
    }
    // clang-format on

    // set the Measurement Covariance Noise Matrix (R) and outlier bounds vector for Kalman filter
    this->set_parameter(rclcpp::Parameter("kalman_filter.R_init", R_int));
    this->set_parameter(rclcpp::Parameter("kalman_filter.outlier_bounds", outlier_bounds));

    // set mapping from identifier to measurement names for logging
    state_estimation_->set_input_config(identifier_names);

    // initialize the topic watchdog and the node monitor
    monitor_ = std::make_unique<tam::core::NodeMonitor>(this);
    topic_watchdog_ = std::make_unique<tam::core::TopicWatchdog>(this);

    // generate delay compensation module
    delay_compensation_ = std::make_unique<tam::helpers::DelayCompensation>(1 / TConfig::TS, TConfig::TS);

    // model step callback
    model_update_timer_ = tam::create_timer(this, std::chrono::microseconds(static_cast<uint64_t>(TConfig::TS * 1e6)),
      std::bind(&stateEstimationNode::function_queue_callback, this));

    // measurement subscriber that has a reference to the configuration
    for (size_t i = 0; i < measurement_channels_.size(); ++i) {
      auto& measurement = measurement_channels_[i];
      if (measurement.config.status_sub.active && measurement.config.measurement_sub.active) {
        // Synchronized subscriptions with diagnostics
        auto sub_odometry = topic_watchdog_->add_synced_subscription<nav_msgs::msg::Odometry>(
          measurement.config.measurement_sub.topic, tam::ros::get_qos(tam::ros::TopicType::DEFAULT));
        auto sub_diagnostic = topic_watchdog_->add_synced_subscription<diagnostic_msgs::msg::DiagnosticArray>(
          measurement.config.status_sub.topic, tam::ros::get_qos(tam::ros::TopicType::DEFAULT));
        topic_watchdog_->register_synced_callback(
          sub_odometry, sub_diagnostic,
          [this, i](const nav_msgs::msg::Odometry::ConstSharedPtr msg_odometry,
            const diagnostic_msgs::msg::DiagnosticArray::ConstSharedPtr msg_diagnostic) {
            this->measurement_status_callback(msg_diagnostic, this->measurement_channels_[i]);
            this->measurement_callback(msg_odometry, this->measurement_channels_[i]);
          },
          [this, i](bool is_timeout, std::chrono::milliseconds elapsed) {
            this->measurement_timeout_callback(is_timeout, elapsed, this->measurement_channels_[i]);
          },
          std::chrono::milliseconds(measurement.config.measurement_sub.timeout_ms));
        continue;
      }
      if (measurement.config.measurement_sub.active)
        topic_watchdog_->add_subscription<nav_msgs::msg::Odometry>(
          measurement.config.measurement_sub.topic, tam::ros::get_qos(tam::ros::TopicType::DEFAULT),
          [this, i](const nav_msgs::msg::Odometry::ConstSharedPtr msg) {
            this->measurement_callback(msg, this->measurement_channels_[i]);
          },
          [this, i](bool is_timeout, std::chrono::milliseconds elapsed) {
            this->measurement_timeout_callback(is_timeout, elapsed, this->measurement_channels_[i]);
          },
          std::chrono::milliseconds(measurement.config.measurement_sub.timeout_ms));
    }

    // input subscriber that has a reference to the configuration
    for (size_t i = 0; i < input_channels_.size(); ++i) {
      auto& input = input_channels_[i];
      if (input.config.measurement_sub.active) {
        topic_watchdog_->add_subscription<sensor_msgs::msg::Imu>(
          input.config.measurement_sub.topic, tam::ros::get_qos(tam::ros::TopicType::DEFAULT),
          [this, i](
            const sensor_msgs::msg::Imu::ConstSharedPtr msg) { this->input_callback(msg, this->input_channels_[i]); },
          [this, i](bool is_timeout, std::chrono::milliseconds elapsed) {
            this->imu_timeout_callback(is_timeout, elapsed, this->input_channels_[i]);
          },
          std::chrono::milliseconds(static_cast<long>(input.config.measurement_sub.timeout_ms)));
      }
    }

    if constexpr (TModel != tam::core::state::VehicleModel::Kinematic) {
      topic_watchdog_->add_subscription<tum_msgs::msg::TUMFloat64PerWheelStamped>("/vehicle/sensor/wheelspeed_radps",
        tam::ros::get_qos(tam::ros::TopicType::DEFAULT),
        std::bind(&stateEstimationNode::wheelspeed_report_callback, this, _1),
        std::bind(&stateEstimationNode::wheelspeed_report_timeout_callback, this, _1, _2), 500ms);

      topic_watchdog_->add_subscription<diagnostic_msgs::msg::DiagnosticArray>("/vehicle/sensor/wheelspeed_status",
        tam::ros::get_qos(tam::ros::TopicType::DEFAULT),
        std::bind(&stateEstimationNode::wheelspeed_status_callback, this, _1),
        std::bind(&stateEstimationNode::wheelspeed_status_timeout_callback, this, _1, _2), 500ms);
    }

    if constexpr (TModel == tam::core::state::VehicleModel::SingleTrack) {
      topic_watchdog_->add_subscription<autoware_auto_vehicle_msgs::msg::SteeringReport>(
        "/vehicle/sensor/steering_report", tam::ros::get_qos(tam::ros::TopicType::DEFAULT),
        std::bind(&stateEstimationNode::steering_report_callback, this, _1),
        std::bind(&stateEstimationNode::steering_report_timeout_callback, this, _1, _2), 500ms);
    }

    // publisher for the state estimation output
    pub_odometry_ = this->create_publisher<nav_msgs::msg::Odometry>(
      "/core/state/odometry", tam::ros::get_qos(tam::ros::TopicType::DEFAULT));

    pub_acceleration_ = this->create_publisher<geometry_msgs::msg::AccelWithCovarianceStamped>(
      "/core/state/acceleration", tam::ros::get_qos(tam::ros::TopicType::DEFAULT));
  }

private:
  /**
   * @brief Declare and update the parameters of the state estimation node
   */
  void declare_and_update_parameters(void)
  {
    // define node specific parameters
    // clang-format off
    state_estimation_node_params_.odom_frame_id = param_manager_->declare_and_get_value("node.odom_frame_id", std::string("local_cartesian"), tam::pmg::ParameterType::STRING, "Frame of the odometry output").as_string(); // NOLINT
    state_estimation_node_params_.child_frame_id = param_manager_->declare_and_get_value("node.child_frame_id", std::string("base_link"), tam::pmg::ParameterType::STRING, "Child frame of the odometry output").as_string(); // NOLINT
    state_estimation_node_params_.virtual_covariance_scale = param_manager_->declare_and_get_value("kalman_filter.covariance_adaption.virtual_scale", 10.0, tam::pmg::ParameterType::DOUBLE, "Scaling Factor for the covariance of delayed measurements").as_double(); // NOLINT
    state_estimation_node_params_.average_input_delay_ms = param_manager_->declare_and_get_value("inputs.average_delay_ms", 0, tam::pmg::ParameterType::INTEGER, "Average expected delay on the IMU measurements").as_int(); // NOLINT
    previous_param_state_hash_ = param_manager_->get_state_hash();
    // clang-format on
  }

  // Timer callbacks
  /**
   * @brief Function queue to handle the node monitor, topic watchdog and state estimation step
   */
  void function_queue_callback(void)
  {
    if (param_manager_->get_state_hash() != previous_param_state_hash_) {
      this->declare_and_update_parameters();
    }
    topic_watchdog_->check_timeouts();
    model_update_callback();
    std_msgs::msg::Header header;
    header.stamp = this->time_pub_;
    header.frame_id = "local_cartesian";
    monitor_->update(header);
  }

  /**
   * @brief Step the state estimation once and publish the outputs
   */
  void model_update_callback(void)
  {
    // iterate through the odometry buffer and input the delay compensated odmetry measurements
    const auto start_delay_comp = std::chrono::high_resolution_clock::now();
    const double covariance_scale = state_estimation_node_params_.virtual_covariance_scale;
    for (const auto& measurement : measurement_channels_) {
      if (measurement.received) {
        tam::types::control::Odometry input = measurement.odometry;
        // compensate the message delay and use the measurement input if the message isn't to old
        // clang-format off
        const double time_diff = static_cast<double>((this->now() - measurement.measurement_stamp).nanoseconds()) / 1e9; // NOLINT
        // clang-format on
        if (delay_compensation_->compensate(input, time_diff)) {
          // increase the covariance of the measurement due to the delay compensation
          std::transform(measurement.odometry.pose_covariance.begin(), measurement.odometry.pose_covariance.end(),
            input.pose_covariance.begin(),
            [covariance_scale, time_diff](double x) { return x + x * covariance_scale * time_diff; });
          // set state estimation localization input
          if (measurement.config.position_m.active)
            state_estimation_->set_input_position(input, measurement.config.position_m.num);
          if (measurement.config.orientation_rad.active)
            state_estimation_->set_input_orientation(input, measurement.config.orientation_rad.num,
              measurement.config.orientation_rad.mask[0], measurement.config.orientation_rad.mask[1],
              measurement.config.orientation_rad.mask[2]);
          if (measurement.config.linear_velocity_mps.active)
            state_estimation_->set_input_linear_velocity(input, measurement.config.linear_velocity_mps.num);
        } else {
          // set sensor state to error if the message is older than a second
          if (measurement.config.position_m.active)
            state_estimation_->set_input_position_status(
              tam::types::ErrorLvl::ERROR, measurement.config.position_m.num);
          if (measurement.config.orientation_rad.active)
            state_estimation_->set_input_orientation_status(
              tam::types::ErrorLvl::ERROR, measurement.config.orientation_rad.num);
          if (measurement.config.linear_velocity_mps.active)
            state_estimation_->set_input_linear_velocity_status(
              tam::types::ErrorLvl::ERROR, measurement.config.linear_velocity_mps.num);
        }
      }
    }
    // runtime logging for the delay compensation
    const auto end_delay_comp = std::chrono::high_resolution_clock::now();
    const double duration_delay_comp_us = std::chrono::duration_cast<std::chrono::microseconds>(
      end_delay_comp - start_delay_comp).count();

    // initialize the initial position and heading of the state estimation
    // set set_initial_state will return true if the state was set successfull
    // we dont publish any message before the state estimation was initialized successfully
    if (initialize_) {
      if (state_estimation_->set_initial_state()) {
        initialize_ = false;
        RCLCPP_INFO(this->get_logger(), "Initial state of the state estimation has been set");
        monitor_->initialization_finished();
      }
      monitor_->set_message(state_estimation_->get_state_machine_status_msg());
    } else {
      // the state estimation was initialized successfully
      // step the state estimation
      const auto start_step = std::chrono::high_resolution_clock::now();
      state_estimation_->step();
      const auto end_step = std::chrono::high_resolution_clock::now();
      const double duration_step_us = std::chrono::duration_cast<std::chrono::microseconds>(
        end_step - start_step).count();

      // update the error level of the state estimatiion node
      const auto start_publish = std::chrono::high_resolution_clock::now();

      monitor_->set_error_lvl("internal state", state_estimation_->get_status());
      monitor_->set_message(state_estimation_->get_state_machine_status_msg());

      // get the state estimation output
      odometry_output_ = state_estimation_->get_odometry();
      acceleration_output_ = state_estimation_->get_acceleration();

      // Publish estimated states with a fixed, non-trivially compensatable delay
      // on the IMU and linear velocity measurements.
      this->time_pub_ = get_clock()->now();
      this->time_pub_ -= std::chrono::milliseconds(state_estimation_node_params_.average_input_delay_ms);

      // publish state estimation output
      publish_odometry(time_pub_);
      publish_acceleration(time_pub_);

      // publish dynamic transforms
      publish_dynamic_transforms(time_pub_);

      // runtime logging for the publisher
      const auto end_publish = std::chrono::high_resolution_clock::now();
      const double duration_publish_us = std::chrono::duration_cast<std::chrono::microseconds>(
        end_publish - start_publish).count();

      // Update the Debug Values of the state estimation class
      const auto start_logging = std::chrono::high_resolution_clock::now();
      state_estimation_->get_state_machine_debug_output(logger_);
      state_estimation_->get_kalman_filter_debug_output(logger_);

      // iterate through the odometry buffer and publish the debug information
      for (const auto& measurement : measurement_channels_) log_measurement_input(measurement);
      const auto end_logging = std::chrono::high_resolution_clock::now();
      const double duration_logging_us = std::chrono::duration_cast<std::chrono::microseconds>(
        end_logging - start_logging).count();
      // time between callbacks
      const rclcpp::Time callback_time = this->get_clock()->now();
      const double callback_dt_us = static_cast<double>(
          (callback_time - last_callback_time_).to_chrono<std::chrono::microseconds>().count()
      );
      last_callback_time_ = callback_time;

      // runtime logging
      logger_->log("runtime/delay_compensation", duration_delay_comp_us);
      logger_->log("runtime/step", duration_step_us);
      logger_->log("runtime/publish", duration_publish_us);
      logger_->log("runtime/logging", duration_logging_us);
      logger_->log("runtime/callback_dt", callback_dt_us);

      // publish the debug message
      tsl_publisher_.trigger();

      // update the delay compensation buffer
      delay_compensation_->update_vehicle_odometry(odometry_output_);
    }
  }

  // publisher function
  /**
   * @brief publish odometry output of the state estimation as nav_msgs::msg::Odometry
   */
  void publish_odometry(rclcpp::Time time_pub)
  {
    // convert odometry type to a nav_msgs::msg::Odometry
    nav_msgs::msg::Odometry odometry_msg = tam::type_conversions::odometry_msg_from_type(odometry_output_);

    // construct message header
    odometry_msg.header.stamp = time_pub;
    odometry_msg.header.frame_id = state_estimation_node_params_.odom_frame_id;
    odometry_msg.child_frame_id = state_estimation_node_params_.child_frame_id;

    pub_odometry_->publish(odometry_msg);
  }

  /**
   * @brief publish linear acceleration output of the state estimation as
   *        geometry_msgs::msg::AccelWithCovarianceStamped
   */
  void publish_acceleration(rclcpp::Time time_pub)
  {
    // convert odometry type to a geometry_msgs::msg::AccelWithCovarianceStamped
    geometry_msgs::msg::AccelWithCovarianceStamped acceleration_msg =
      tam::type_conversions::accel_with_covariance_stamped_msg_from_type(acceleration_output_);

    // construct message header
    acceleration_msg.header.stamp = time_pub;
    acceleration_msg.header.frame_id = "vehicle_cg";

    pub_acceleration_->publish(acceleration_msg);
  }

  /**
   * @brief publish dynamic transforms:
   *            local_cartesian -> vehicle_velocity_cg
   *            vehicle_cg      -> vehicle_velocity_cg
   */
  void publish_dynamic_transforms(rclcpp::Time time_pub)
  {
    // get timestamp and publish transform from
    // local_cartesian to base_link
    geometry_msgs::msg::TransformStamped transform_local_cartesian;
    transform_local_cartesian.header.stamp = time_pub;
    transform_local_cartesian.header.frame_id = state_estimation_node_params_.odom_frame_id;
    transform_local_cartesian.child_frame_id = state_estimation_node_params_.child_frame_id;

    // translation between current cog and 0, 0, 0
    transform_local_cartesian.transform.translation.x = odometry_output_.position_m.x;
    transform_local_cartesian.transform.translation.y = odometry_output_.position_m.y;
    transform_local_cartesian.transform.translation.z = odometry_output_.position_m.z;

    // rotate yaw angle around z
    transform_local_cartesian.transform.rotation =
      tam::types::conversion::euler_type_to_quaternion_msg(tam::types::common::EulerYPR(
        odometry_output_.orientation_rad.z, odometry_output_.orientation_rad.y, odometry_output_.orientation_rad.x));

    // publish the transformation to the vehicle CoG
    tf_broadcaster_->sendTransform(transform_local_cartesian);

    // get timestamp and publish transform from
    // vehicle_cg to vehicle_velocity_cg
    geometry_msgs::msg::TransformStamped transform_vehicle_cg;
    transform_vehicle_cg.header.stamp = time_pub;
    transform_vehicle_cg.header.frame_id = "vehicle_cg";
    transform_vehicle_cg.child_frame_id = "vehicle_velocity_cg";

    // there is no translation
    transform_vehicle_cg.transform.translation.x = 0.0;
    transform_vehicle_cg.transform.translation.y = 0.0;
    transform_vehicle_cg.transform.translation.z = 0.0;

    // rotate sideslip angle around z
    transform_vehicle_cg.transform.rotation = tam::types::conversion::euler_type_to_quaternion_msg(
      tam::types::common::EulerYPR(state_estimation_->get_sideslip_angle(), 0.0, 0.0));

    // publish the transformation for sideslip angle
    tf_broadcaster_->sendTransform(transform_vehicle_cg);
  }

  /**
   * @brief publish the transformed odometry input
   *
   * @param[in] measurement         - tam::types::state::measurement:
   *                                  struct containing all relevant information about
   *                                  the received measurement
   */
  void log_measurement_input(const tam::types::state::measurement& measurement)
  {
    if (!measurement.config.position_m.active || !measurement.config.orientation_rad.active) return;

    // publish the delay compensated and transformed measurements as debug
    std::string debug_namespace = "pose/" + measurement.config.name;
    debug_namespace.reserve(debug_namespace.size() + 64);

    // lamda to log the transformed odometry as debug
    auto log_odometry = [&](const std::string& prefix, const tam::types::control::Odometry& odom, double delay) {
      logger_->log(debug_namespace + prefix + "/position_m/x", odom.position_m.x);
      logger_->log(debug_namespace + prefix + "/position_m/y", odom.position_m.y);
      logger_->log(debug_namespace + prefix + "/position_m/z", odom.position_m.z);
      logger_->log(debug_namespace + prefix + "/orientation_rad/roll", odom.orientation_rad.x);
      logger_->log(debug_namespace + prefix + "/orientation_rad/pitch", odom.orientation_rad.y);
      logger_->log(debug_namespace + prefix + "/orientation_rad/yaw", odom.orientation_rad.z);
      logger_->log(debug_namespace + prefix + "/velocity_mps/x", odom.velocity_mps.x);
      logger_->log(debug_namespace + prefix + "/velocity_mps/y", odom.velocity_mps.y);
      logger_->log(debug_namespace + prefix + "/velocity_mps/z", odom.velocity_mps.z);
      logger_->log(debug_namespace + prefix + "/msg_delay", delay);
    };

    if (measurement.received) {
      // once delay compensated measurements
      tam::types::control::Odometry delay_compensated_odometry = measurement.odometry;
      const double measured_time_diff =
        static_cast<double>((measurement.received_stamp - measurement.measurement_stamp).nanoseconds()) / 1e9;
      delay_compensation_->compensate(delay_compensated_odometry, measured_time_diff);
      log_odometry("/measured", delay_compensated_odometry, measured_time_diff);

      // Continously forward predicted virtual measurements
      tam::types::control::Odometry virtual_odometry = measurement.odometry;
      const double virtual_time_diff =
        static_cast<double>((this->now() - measurement.measurement_stamp).nanoseconds()) / 1e9;
      delay_compensation_->compensate(virtual_odometry, virtual_time_diff);
      log_odometry("/virtual", virtual_odometry, virtual_time_diff);
    } else {
      log_odometry("/measured", {}, 0.0);
      log_odometry("/virtual", {}, 0.0);
    }
  }

  // Subscription callbacks sensors
  void measurement_callback(
    const nav_msgs::msg::Odometry::ConstSharedPtr msg, tam::types::state::measurement& measurement)
  {
    // input position of the first sensor position
    measurement.odometry = tam::type_conversions::odometry_type_from_msg(*msg);

    // Transform pose to center of gravity
    if (measurement.config.position_m.active || measurement.config.orientation_rad.active) {
      try {
        // Try to look up the transform until the first successful lookup
        if (!measurement.static_translation_valid) {
          geometry_msgs::msg::TransformStamped transform = tf_buffer_->lookupTransform(
            msg->child_frame_id, state_estimation_node_params_.child_frame_id, tf2::TimePointZero);
          measurement.static_translation = tf2::Vector3(transform.transform.translation.x,
            transform.transform.translation.y, transform.transform.translation.z);
          measurement.static_translation_valid = true;
        }

        // build the rotation matrix
        tf2::Matrix3x3 rotation_matrix;
        if (initialize_) {
          rotation_matrix.setEulerYPR(measurement.odometry.orientation_rad.z, measurement.odometry.orientation_rad.y,
            measurement.odometry.orientation_rad.x);
        } else {
          rotation_matrix.setEulerYPR(
            odometry_output_.orientation_rad.z, odometry_output_.orientation_rad.y, odometry_output_.orientation_rad.x);
        }

        // rotate translation in the local cartesian frame
        const tf2::Vector3 transformation = rotation_matrix * measurement.static_translation;

        // transform from sensor frame to vehicle CoG
        measurement.odometry.position_m = measurement.odometry.position_m +
          tam::types::common::Vector3D<double>{transformation.x(), transformation.y(), transformation.z()};
      } catch (tf2::TransformException& ex) {
        RCLCPP_INFO(this->get_logger(), "Failed to transform: %s", ex.what());
        return;
      }
    }
    // buffer the received odometry and account for constant delay
    rclcpp::Time measurement_timestamp = msg->header.stamp;

    // Compute the additional delay
    const auto additional_odom_delay_ms = std::chrono::milliseconds(measurement.config.additional_delay);
    const auto average_input_delay_ms = std::chrono::milliseconds(state_estimation_node_params_.average_input_delay_ms);
    if ((measurement_timestamp - additional_odom_delay_ms + average_input_delay_ms) <= this->now()) {
      measurement_timestamp -= additional_odom_delay_ms - average_input_delay_ms;
    } else {
      measurement_timestamp -= additional_odom_delay_ms;
    }

    // set timestamps in buffer
    measurement.measurement_stamp = measurement_timestamp;
    measurement.received_stamp = this->now();
    measurement.received = true;
  }

  void measurement_status_callback(
    const diagnostic_msgs::msg::DiagnosticArray::ConstSharedPtr msg, const tam::types::state::measurement& measurement)
  {
    // set state estimation sensor status
    if (msg->status.empty()) return;
    if (measurement.config.position_m.active)
      state_estimation_->set_input_position_status(
        tam::type_conversions::error_type_from_diagnostic_level(msg->status[0].level),
        measurement.config.position_m.num);
    if (measurement.config.orientation_rad.active)
      state_estimation_->set_input_orientation_status(
        tam::type_conversions::error_type_from_diagnostic_level(msg->status[0].level),
        measurement.config.orientation_rad.num);
    if (measurement.config.linear_velocity_mps.active)
      state_estimation_->set_input_linear_velocity_status(
        tam::type_conversions::error_type_from_diagnostic_level(msg->status[0].level),
        measurement.config.linear_velocity_mps.num);
  }

  void input_callback(const sensor_msgs::msg::Imu::ConstSharedPtr msg, tam::types::state::input& input)
  {
    tam::types::control::AccelerationwithCovariances input_acceleration =
      tam::type_conversions::acceleration_with_covariances_type_from_imu_msg(*msg);

    const tam::types::control::Odometry input_odometry = tam::type_conversions::odometry_type_from_imu_msg(*msg);

    // set the imu input valid state OK (0)
    state_estimation_->set_input_imu_status(tam::types::ErrorLvl::OK, input.config.num);

    // set acceleration and angular velocity input
    state_estimation_->set_input_acceleration(input_acceleration, input.config.num);
    state_estimation_->set_input_angular_velocity(input_odometry, input.config.num);

    input.received = true;
  }

  void wheelspeed_report_callback(const tum_msgs::msg::TUMFloat64PerWheelStamped::ConstSharedPtr msg)
  {
    // input angluar velocity per wheel
    const tam::types::common::DataPerWheel<double> input =
      tam::type_conversions::data_per_wheel_type_from_msg(msg->data);

    // set the wheelspeed input
    state_estimation_->set_input_wheelspeeds(input);
  }

  void wheelspeed_status_callback(const diagnostic_msgs::msg::DiagnosticArray::ConstSharedPtr msg)
  {
    if (msg->status.empty()) return;
    // set state estimation sensor status
    state_estimation_->set_input_wheelspeed_status(
      tam::type_conversions::error_type_from_diagnostic_level(msg->status[0].level));
  }

  void steering_report_callback(const autoware_auto_vehicle_msgs::msg::SteeringReport::ConstSharedPtr msg)
  {
    // set the steering angle input and the status
    state_estimation_->set_input_steering_angle(msg->steering_tire_angle);
    state_estimation_->set_input_steering_angle_status(tam::types::ErrorLvl::OK);
  }

  // Timeout callback for the sensor subscription
  void measurement_timeout_callback(bool timeout, [[maybe_unused]] std::chrono::milliseconds timeout_now,
    const tam::types::state::measurement& measurement)
  {
    if (!timeout) return;
    if (measurement.config.position_m.active)
      state_estimation_->set_position_timeout(measurement.config.position_m.num);
    if (measurement.config.orientation_rad.active)
      state_estimation_->set_orientation_timeout(measurement.config.orientation_rad.num);
    if (measurement.config.linear_velocity_mps.active)
      state_estimation_->set_linear_velocity_timeout(measurement.config.linear_velocity_mps.num);
  }

  void imu_timeout_callback(
    bool timeout, [[maybe_unused]] std::chrono::milliseconds timeout_now, const tam::types::state::input& input)
  {
    if (timeout && input.received) {
      state_estimation_->set_imu_timeout(input.config.num);
    }
  }

  void wheelspeed_report_timeout_callback(bool timeout, [[maybe_unused]] std::chrono::milliseconds timeout_now)
  {
    if (!timeout) return;
    state_estimation_->set_wheelspeeds_timeout();
  }

  void wheelspeed_status_timeout_callback(bool timeout, [[maybe_unused]] std::chrono::milliseconds timeout_now)
  {
    if (!timeout) return;
    state_estimation_->set_wheelspeeds_timeout();
  }

  void steering_report_timeout_callback(bool timeout, [[maybe_unused]] std::chrono::milliseconds timeout_now)
  {
    if (!timeout) return;
    state_estimation_->set_steering_angle_timeout();
  }

  /**
   * @brief state estimation class
   */
  std::unique_ptr<tam::core::state::StateEstimationBase> state_estimation_{};

  /**
   * @brief TAM Parameter Manager Composer to combine the parameters of all subclasses
   */
  tam::pmg::ParamManagerComposer::SharedPtr param_manager_composer_{};
  OnSetParametersCallbackHandle::SharedPtr callback_handle_{};

  /**
   * @brief TAM Parameter Manager for the state estimation node
   */
  tam::pmg::ParamValueManager::SharedPtr param_manager_ = std::make_shared<tam::pmg::ParamValueManager>();

  // Variables
  /**
   * @brief Variable containing the previous parameter state hash
   */
  std::size_t previous_param_state_hash_{0};
  /**
   * @brief Struct containing the parameters for the state estimation node
   */
  tam::types::state::config::node state_estimation_node_params_;
  /**
   * @brief model update callback
   */
  rclcpp::TimerBase::SharedPtr model_update_timer_{};
  rclcpp::Time time_pub_{0, 0, RCL_ROS_TIME};

  /**
   * @brief topic watchdog
   */
  tam::core::TopicWatchdog::UniquePtr topic_watchdog_{};

  /**
   * @brief node monitor
   */
  tam::core::NodeMonitor::UniquePtr monitor_{};

  /**
   * @brief delay compensation
   */
  std::unique_ptr<tam::helpers::DelayCompensation> delay_compensation_{};

  /**
   * @brief tf2 transform broadcaster
   */
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_{};

  /**
   * @brief tf2 transform listener
   */
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_{nullptr};
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_{};

  /**
   * @brief initialize the initial state in the state estimation
   */
  bool initialize_{true};

  /**
   * @brief Time of the previous model update callback
   */
  rclcpp::Time last_callback_time_{0, 0, RCL_ROS_TIME};

  // state estimation output
  /**
   * @brief state estimation odometry output
   */
  tam::types::control::Odometry odometry_output_{};

  /**
   * @brief state estimation acceleration output
   */
  tam::types::control::AccelerationwithCovariances acceleration_output_{};

  // Publishers
  /**
   * @brief nav_msgs::msg::Odometry publisher
   */
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr pub_odometry_{};

  /**
   * @brief geometry_msgs::msg::AccelWithCovarianceStamped publisher
   */
  rclcpp::Publisher<geometry_msgs::msg::AccelWithCovarianceStamped>::SharedPtr pub_acceleration_{};

  /**
   * @brief Create Logger
   */
  tam::tsl::ValueLogger::SharedPtr logger_ = std::make_shared<tam::tsl::ValueLogger>();
  tam::tsl::TSLPublisher tsl_publisher_{this, logger_};

  /**
   * @brief array containing all measurement channels
   */
  std::vector<tam::types::state::measurement> measurement_channels_{};

  /**
   * @brief array containing all input channels
   */
  std::vector<tam::types::state::input> input_channels_{};
};
