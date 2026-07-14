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
namespace tam::core::state::types
{
enum class DistortionType : std::uint8_t { POLYNOM = 0, CUDA_POLYNOM = 1 };
/**
 * @brief Configuration for the distortion handler
 */
struct DistortionConfig
{
  double max_time_diff{30};    // ms, max time difference between frame and pose history
  double vel_threshold{10.0};  // m/s, threshold for velocity to enable distortion correction
};
/**
 * @brief Debug signals for the distortion handler
 */
struct DistortionDebug
{
  bool valid{false};
  double undistortion_time{0.0};
  double timestamp_offset{0.0};
  std::int64_t num_invalid_points{0};
};
}  // namespace tam::core::state::types
