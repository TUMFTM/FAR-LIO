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
enum class DiagnosticType : std::uint8_t { BASE = 0 };
/**
 * @brief Configuration for the diagnostics
 */
struct DiagnosticConfig
{
  // Check for input diagnostic status
  bool check_input_status{true};
  // Outdated frame warning
  double frame_outdated{100.0};  // ms
  // Check if pose in front of previous pose
  bool check_forward{true};
  double min_motion_threshold{0.0};
  // Check if pose is within ellipsis around initial guess
  bool check_ellipsis{true};
  double ellipsis_size_s{5.0};
  double ellipsis_size_d{3.0};
  // Check if time diffference between pose and initial guess is within bounds
  bool check_time_diff{true};
  double max_time_diff{0.05};  // s
  // Check if velocity is reasonable
  bool enable_vel{false};
  bool check_vel{true};
  double vel_motion_threshold{0.25};
};
/**
 * @brief Debug signals for the diagnostics
 */
struct DiagnosticDebug
{
  std::int64_t overall_status{0};
  struct registration
  {
    std::int64_t status{0};
  } registration{};
  struct input
  {
    std::int64_t status{0};
  } input{};
  struct pose
  {
    std::int64_t status{0};
    double diff_initial_guess_s{0.0};
    double diff_initial_guess_d{0.0};
    double diff_time{0.0};
  } pose{};
};
}  // namespace tam::core::state::types
