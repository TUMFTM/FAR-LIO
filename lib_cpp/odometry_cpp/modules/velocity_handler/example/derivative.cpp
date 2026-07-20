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

#include "velocity_handler/derivative.hpp"

#include <chrono>
#include <iostream>
#include <variant>

//
int main()
{
  tam::core::state::types::VelocityConfig config;
  tam::core::state::types::VelocityDebug debug;
  auto vh = tam::core::state::Derivative<tam::core::state::types::ICP_EXT>::from_config(config, debug);

  const std::uint64_t t0 = static_cast<std::uint64_t>(1e9);
  const std::uint64_t step = static_cast<std::uint64_t>(0.1e9);
  for (int i = 0; i < 5; ++i) {
    tam::core::state::types::PoseStamped pose;
    pose.stamp = t0 + static_cast<std::uint64_t>(i) * step;
    pose.pose.translation().x() = static_cast<float>(i) * 1.0f;
    const auto result = vh->get_tangent({}, pose);
    const auto& dbg = vh->get_debug();
    const double dt = dbg.conditional.count("vel_dt") ? std::get<double>(dbg.conditional.at("vel_dt")) : 0.0;
    std::cout << "[" << i << "] vx=" << result.tangent(0) << " vy=" << result.tangent(1) << " vz=" << result.tangent(2)
              << " dt=" << dt << " velocity_time=" << dbg.velocity_time << " ms" << std::endl;
  }
  return 0;
}
