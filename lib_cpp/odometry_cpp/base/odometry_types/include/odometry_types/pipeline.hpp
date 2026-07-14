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

#include "preprocessing.hpp"
#include <vector>
namespace tam::core::state::types
{
/**
 * @brief Configuration for the pipeline
 */
struct PipelineConfig
{
  bool update_map{false};
  bool undistort{true};
  bool preprocess{true};
  bool downsample{true};
  bool buffer_z{true};
  bool debug_mode{false};
  std::int64_t num_threads{1};
};
/**
 * @brief Debug signals for the pipeline
 */
struct PipelineDebug
{
  double downsample_time{0.0};
  double pipeline_time{0.0};
  std::int64_t num_points_frame{0};
};
}  // namespace tam::core::state::types
