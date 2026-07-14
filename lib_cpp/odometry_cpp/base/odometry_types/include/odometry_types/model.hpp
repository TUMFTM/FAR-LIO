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
#include <vector>
#include <string>

namespace tam::core::state::types
{
enum class ModelType : std::uint8_t { EXTERNALGUESS = 0, CONSTANTVELOCITY = 1 };
/**
 * @brief Configuration for the model handler
 */
struct ModelConfig
{
  double initial_pos_x{0.0};
  double initial_pos_y{0.0};
  double initial_pos_z{0.0};
  double initial_rot_x{0.0};
  double initial_rot_y{0.0};
  double initial_rot_z{0.0};
  double initial_rot_w{0.0};
};
/**
 * @brief Debug signals for the model handler
 */
struct ModelDebug
{
  double current_pos_x{0.0};
  double current_pos_y{0.0};
  double current_pos_z{0.0};
  double current_rot_x{0.0};
  double current_rot_y{0.0};
  double current_rot_z{0.0};
  double current_rot_w{0.0};
};
}  // namespace tam::core::state::types
