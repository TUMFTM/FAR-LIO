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

#include <string>
#include <unordered_map>
#include <variant>

namespace tam::core::state::types {
enum class RegistrationType : std::uint8_t { ICP = 0, CUDA_ICP = 1, GICP = 2, CUDA_GICP = 3 };
/**
 * @brief Status of the solver after registration
 */
enum class SolverStatus : std::uint8_t { NOT_CONVERGED = 0, CONVERGED = 1, INVALID = 2 };

/**
 * @brief Configuration for the registration handler
 */
struct RegistrationConfig {
  std::string solver_type{"GaussNewton"};
  std::int64_t max_iter{500};
  std::int64_t max_inner_iter{20};  // Only used for LM solver
  double max_time{200.0};
  double convergence_criterion{5.0e-3};
  double damping_factor{0.0};
  double damping_scale{10.0};  // Only used for LM solver
};

/**
 * @brief Debug signals for the registration handler
 */
struct RegistrationDebug {
  bool converged{false};
  double registration_time{0.0};
  double damping_factor{0.0};
  std::int64_t num_points_frame{0};
  std::int64_t num_iter{0};
  std::unordered_map<std::string, std::variant<std::int64_t, double>> conditional{};
};
}  // namespace tam::core::state::types
