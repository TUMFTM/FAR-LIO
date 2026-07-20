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
#include <string>
#include <vector>
namespace tam::core::state
{
struct KF_BASE
{
  /**
   * @brief State Estimation constants
   */
  static inline double TS = 0.01;

  /**
   * @brief Kalman filter base constants (number inputs)
   *        Maximum number of IMU inputs 
   */
  static inline constexpr int NUM_IMU_MEASUREMENT = 3;
  static inline constexpr int NUM_BACKUP_IMU_MEASUREMENT = 1;

  /**
   * @brief Kalman filter base constants (number inputs)
   *        Maximum number of Measurement inputs
   *        ODOMETRY: POSITION, ORIENTATION, LINEAR VELOCITY
   *        POSE:     POSITION, ORIENTATION
   *        VEL:      LINEAR VELOCITY
   */
  static inline constexpr int NUM_ODOMETRY_MEASUREMENT = 2;
  static inline constexpr int NUM_POSE_MEASUREMENT = 2;
  static inline constexpr int NUM_VEL_SENSOR_MEASUREMENTS = 2;

  /**
   * @brief Kalman filter base constants (number inputs)
   *        NUM_ORIENTATION_MEASUREMENT should always be N + 2 due to internal variable handling
   */
  static inline constexpr int NUM_POS_MEASUREMENT = NUM_ODOMETRY_MEASUREMENT + NUM_POSE_MEASUREMENT;
  static inline constexpr int NUM_ORIENTATION_MEASUREMENT =  NUM_ODOMETRY_MEASUREMENT + NUM_POSE_MEASUREMENT + 2;

  /**
   * @brief Kalman filter base constants (number inputs)
   *        NUM_VEL_MEASUREMENT should always be N + 1 (Internal Model) due to internal
   * variable handling
   */
  static inline constexpr int NUM_VEL_MEASUREMENT = NUM_VEL_SENSOR_MEASUREMENTS + NUM_ODOMETRY_MEASUREMENT + 1;
};
}  // namespace tam::core::state
