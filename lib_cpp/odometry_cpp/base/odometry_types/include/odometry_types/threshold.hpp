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

namespace tam::core::state::types {
enum class ThresholdType : std::uint8_t { FIXEDTHRESHOLD = 0, ADAPTIVETHRESHOLD = 1 };

/**
 * @brief Configuration for the threshold handler
 */
struct ThresholdConfig {
  double initial_threshold{0.0};
  double min_motion_threshold{0.0};
  double max_correspondence_range{0.0};
};

/**
 * @brief Debug signals for the threshold handler
 */
struct ThresholdDebug {
  double internal_threshold{0.0};
  double current_threshold{0.0};
  double model_error{0.0};
};
}  // namespace tam::core::state::types
