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

#include <sophus/se3.hpp>

namespace tam::core::state::utils
{
/**
 * @brief Predict pose based on constant velocity assumption (CV)
 * @note It's actually constant distance
 * @param current Current pose
 * @param previous Previous pose
 * @return Predicted pose
 */
inline Sophus::SE3f cv_prediction(const Sophus::SE3f & current, const Sophus::SE3f & previous)
{
  return current * previous.inverse() * current;
}
}  // namespace tam::core::state::utils
