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
#include <gtest/gtest.h>

#include <chrono>

#include "state_estimation_constants/EKF_2D.hpp"
#include "state_estimation_constants/EKF_3D.hpp"
#include "tam_state_estimation/state_estimation.hpp"
#include "tum_types_cpp/common.hpp"

TEST(ExecutionUnitTests, ExecutionTimeEKF2D)
{
  // Create an instance of tam::core::state::StateEstimation
  tam::core::state::StateEstimation<tam::core::state::EKF_2D, tam::core::state::VehicleModel::NonHolonomic>
    StateEstimation;
  tam::tsl::ValueLogger::SharedPtr logger = std::make_shared<tam::tsl::ValueLogger>();
  int64_t execution_time = 0;

  // set the sensor status to initialize the state machine
  StateEstimation.set_input_position_status(tam::types::ErrorLvl::OK, 0);
  StateEstimation.set_input_orientation_status(tam::types::ErrorLvl::OK, 0);
  StateEstimation.set_input_imu_status(tam::types::ErrorLvl::OK, 0);
  StateEstimation.set_input_imu_status(tam::types::ErrorLvl::OK, 1);
  StateEstimation.set_input_wheelspeed_status(tam::types::ErrorLvl::OK);

  // set a random odometry and wheelspeed to allow the initial state to be set
  tam::types::control::Odometry input_odometry{};
  StateEstimation.set_input_position(input_odometry, 0);
  StateEstimation.set_input_orientation(input_odometry, 0, true, true, true);

  tam::types::common::DataPerWheel<double> input_wheelspeed{};
  StateEstimation.set_input_wheelspeeds(input_wheelspeed);

  StateEstimation.set_initial_state();

  for (int i = 0; i < 1000; i++) {
    // start time of the function execution
    auto start = std::chrono::high_resolution_clock::now();

    // set the sensor status to initialize the state machine
    StateEstimation.set_input_position_status(tam::types::ErrorLvl::OK, 0);
    StateEstimation.set_input_orientation_status(tam::types::ErrorLvl::OK, 0);
    StateEstimation.set_input_imu_status(tam::types::ErrorLvl::OK, 0);
    StateEstimation.set_input_imu_status(tam::types::ErrorLvl::OK, 1);
    StateEstimation.set_input_wheelspeed_status(tam::types::ErrorLvl::OK);

    // set a random odometry and wheelspeed to allow the initial state to be set
    tam::types::control::Odometry input_odometry{};
    StateEstimation.set_input_position(input_odometry, 0);
    StateEstimation.set_input_orientation(input_odometry, 0, true, true, true);

    tam::types::common::DataPerWheel<double> input_wheelspeed{};
    StateEstimation.set_input_wheelspeeds(input_wheelspeed);

    // one step of the state estimation
    StateEstimation.step();

    // get the state estimation output
    StateEstimation.get_status();
    StateEstimation.get_odometry();
    StateEstimation.get_acceleration();
    StateEstimation.get_state_machine_debug_output(logger);
    StateEstimation.get_kalman_filter_debug_output(logger);

    // end time
    auto end = std::chrono::high_resolution_clock::now();

    // Calculate the execution time in microseconds
    execution_time += std::chrono::duration_cast<std::chrono::microseconds>(end - start).count();
  }

  // run the test (divide by 1000 to get the mean)
  // one step souldn't take more than 200 microseconds
  EXPECT_LE(execution_time / 1e3, 200);
}

TEST(ExecutionUnitTests, ExecutionTimeEKF3D)
{
  // Create an instance of tam::core::state::StateEstimation
  tam::core::state::StateEstimation<tam::core::state::EKF_3D, tam::core::state::VehicleModel::NonHolonomic>
    StateEstimation;
  tam::tsl::ValueLogger::SharedPtr logger = std::make_shared<tam::tsl::ValueLogger>();
  int64_t execution_time = 0;

  // set the sensor status to initialize the state machine
  StateEstimation.set_input_position_status(tam::types::ErrorLvl::OK, 0);
  StateEstimation.set_input_orientation_status(tam::types::ErrorLvl::OK, 0);
  StateEstimation.set_input_imu_status(tam::types::ErrorLvl::OK, 0);
  StateEstimation.set_input_imu_status(tam::types::ErrorLvl::OK, 1);
  StateEstimation.set_input_wheelspeed_status(tam::types::ErrorLvl::OK);

  // set a random odometry and wheelspeed to allow the initial state to be set
  tam::types::control::Odometry input_odometry{};
  StateEstimation.set_input_position(input_odometry, 0);
  StateEstimation.set_input_orientation(input_odometry, 0, true, true, true);

  tam::types::common::DataPerWheel<double> input_wheelspeed{};
  StateEstimation.set_input_wheelspeeds(input_wheelspeed);

  StateEstimation.set_initial_state();

  for (int i = 0; i < 1000; i++) {
    // start time of the function execution
    auto start = std::chrono::high_resolution_clock::now();

    // set the sensor status to initialize the state machine
    StateEstimation.set_input_position_status(tam::types::ErrorLvl::OK, 0);
    StateEstimation.set_input_orientation_status(tam::types::ErrorLvl::OK, 0);
    StateEstimation.set_input_imu_status(tam::types::ErrorLvl::OK, 0);
    StateEstimation.set_input_imu_status(tam::types::ErrorLvl::OK, 1);
    StateEstimation.set_input_wheelspeed_status(tam::types::ErrorLvl::OK);

    // set a random odometry and wheelspeed to allow the initial state to be set
    tam::types::control::Odometry input_odometry{};
    StateEstimation.set_input_position(input_odometry, 0);
    StateEstimation.set_input_orientation(input_odometry, 0, true, true, true);

    tam::types::common::DataPerWheel<double> input_wheelspeed{};
    StateEstimation.set_input_wheelspeeds(input_wheelspeed);

    // one step of the state estimation
    StateEstimation.step();

    // get the state estimation output
    StateEstimation.get_status();
    StateEstimation.get_odometry();
    StateEstimation.get_acceleration();
    StateEstimation.get_state_machine_debug_output(logger);
    StateEstimation.get_kalman_filter_debug_output(logger);

    // end time
    auto end = std::chrono::high_resolution_clock::now();

    // Calculate the execution time in microseconds
    execution_time += std::chrono::duration_cast<std::chrono::microseconds>(end - start).count();
  }

  // run the test (divide by 1000 to get the mean)
  // one step souldn't take more than 200 microseconds
  EXPECT_LE(execution_time / 1e3, 200);
}
