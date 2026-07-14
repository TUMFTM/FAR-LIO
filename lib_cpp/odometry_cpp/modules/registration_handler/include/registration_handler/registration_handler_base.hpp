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

#ifdef __CUDACC__
#include <cuda_runtime.h>
#include <thrust/device_vector.h>

#include <cuda/stream_ref>

#include "map_handler/voxel_hash_map.cuh"
#include "registration_handler/registration_utils.cuh"

#endif

#include <tbb/blocked_range.h>
#include <tbb/concurrent_vector.h>
#include <tbb/global_control.h>
#include <tbb/info.h>
#include <tbb/parallel_for.h>
#include <tbb/parallel_reduce.h>
#include <tbb/task_arena.h>

#include <algorithm>
#include <chrono>
#include <eigen3/Eigen/Core>
#include <memory>
#include <sophus/se3.hpp>
#include <vector>

#include "map_handler/map_handler_base.hpp"
#include "map_handler/voxel_hash_map.hpp"
#include "odometry_base/odometry_base.hpp"
#include "registration_handler/factor_base.hpp"
#include "registration_handler/registration_utils.hpp"
#include "robust_kernel/robust_kernel.hpp"
namespace tam::core::state
{
template <typename TConfig>
class RegistrationHandler
: public OdometryBase<TConfig, types::RegistrationConfig, types::RegistrationDebug>
{
public:
/**
 * @brief Register a frame to the map
 * @param [in] frame                    Frame to register
 * @param [in] map                      Map to register to
 * @param [in] initial_guess            Initial guess for the registration
 * @param [in] correspondence_threshold Correspondence threshold
 * @param [in] kernel_scale             Scale of the robust kernel
 * @return                              Transformation from frame to map
 */
#ifdef __CUDACC__
  virtual __host__ Sophus::SE3f register_frame(
    const thrust::device_vector<types::Point<TConfig>> & frame, const MapHandler<TConfig> * map,
    const Sophus::SE3f & initial_guess, const float correspondence_threshold,
    const float kernel_scale, ::cuda::stream_ref stream = {}) = 0;
#else
  virtual Sophus::SE3f register_frame(
    const std::vector<types::Point<TConfig>> & frame, const MapHandler<TConfig> * map,
    const Sophus::SE3f & initial_guess, const float correspondence_threshold,
    const float kernel_scale) = 0;
#endif
  /**
   * @brief Get the registration status
   * @return                        Registration status
   */
  types::RegistrationStatus get_registration_status() const
  {
    types::RegistrationStatus status{};
    status.converged = this->debug_.converged;
    status.duration = this->debug_.registration_time;
    status.num_iter = this->debug_.num_iter;
    return status;
  };
/**
 * @brief Get correspondences between map and frame for given pose
 * @param [in] points                     Frame with points
 * @param [in] map                        pointer to map handler
 * @param [in] pose                       pose to get correspondences for
 * @param [in] correspondence_threshold   threshold for computation
 * @return vector of correspondences
 */
#ifdef __CUDACC__
  virtual __host__ std::vector<types::Correspondence<TConfig>> get_correspondences(
    const thrust::device_vector<types::Point<TConfig>> & points, const MapHandler<TConfig> * map,
    const Sophus::SE3f & pose, const float correspondence_threshold,
    ::cuda::stream_ref stream = {}) const = 0;
#else
  virtual std::vector<types::Correspondence<TConfig>> get_correspondences(
    const std::vector<types::Point<TConfig>> & points, const MapHandler<TConfig> * map,
    const Sophus::SE3f & pose, const float correspondence_threshold) const = 0;
#endif
  /**
   * @brief Initialize threading
   * -> needs to be done after param override
   */
  void init(const std::size_t num_threads = 1)
  {
    // This global variable requires static duration storage to be able to manipulate the max
    // concurrency from TBB across the entire class
    static const auto tbb_control_settings = tbb::global_control(
      tbb::global_control::max_allowed_parallelism, static_cast<size_t>(num_threads));
  }
  /**
   * @brief Initialize frame map instance if necessary
   * @param [in] config Configuration for the map
   * @note Debug signal of frame map are currently not used
   */
  void init_frame_map(const types::MapConfig & config) requires types::FRAMEMAP<TConfig>
  {
    // Initialize map handler instance for frame
    // Copy the config to avoid modifying the original
    types::MapConfig map_config = config;
    // Set the frame map config
    map_config.frame_map = true;
    types::MapDebug map_debug{};
    INIT_MODULE(MAP, MapType::VOXELHASHMAP, frame_map_, VoxelHashMap, map_config, map_debug);
#ifdef __CUDACC__
    INIT_MODULE(
      MAP, MapType::CUDA_VOXELHASHMAP, frame_map_, cuda::VoxelHashMap, map_config, map_debug);
#endif
    this->frame_map_->init(false);
  }
  /**
   * @brief Get the average number of points per voxel in the frame map
   */
#ifdef __CUDACC__
  types::AdaptiveMapDensity get_frame_map_density(
    const double range, const Eigen::Vector3f & origin,
    ::cuda::stream_ref stream = {}) requires types::FRAMEMAP<TConfig>
  {
    // clang-format off
    types::AdaptiveMapDensity density = this->frame_map_->get_density(range, stream);  // NOLINT
    density.origin = origin;
    this->debug_.conditional["frame_map_max_points"] = static_cast<int64_t>(density.max_points);
    this->debug_.conditional["frame_map_min_points"] = static_cast<int64_t>(density.min_points);
    this->debug_.conditional["frame_map_max_points_scale"] = static_cast<double>(density.max_points_scale);  // NOLINT
    return density;
  }
#else
  types::AdaptiveMapDensity get_frame_map_density(
    const double range, const Eigen::Vector3f & origin) requires types::FRAMEMAP<TConfig>
  {
    types::AdaptiveMapDensity density = this->frame_map_->get_density(range);  // NOLINT
    density.origin = origin;
    this->debug_.conditional["frame_map_max_points"] = static_cast<int64_t>(density.max_points);
    this->debug_.conditional["frame_map_min_points"] = static_cast<int64_t>(density.min_points);
    this->debug_.conditional["frame_map_max_points_scale"] = static_cast<double>(density.max_points_scale);  // NOLINT
    return density;
    // clang-format on
  }
#endif

protected:
  // Inherit constructor from OdometryBase for param manager and logger
  RegistrationHandler(tam::pmg::ParamReferenceManager * pmg, tam::tsl::ReferenceLogger * logger)
  : OdometryBase<TConfig, types::RegistrationConfig, types::RegistrationDebug>(pmg, logger)
  {
    this->set_config(pmg);
    this->set_logging(logger);
    // Additional initialization
#ifdef __CUDACC__
    this->setup_cuda_device();
#endif
  }
  // Inherit constructor from OdometryBase for config and debug object
  RegistrationHandler(
    const types::RegistrationConfig & config, const types::RegistrationDebug & debug)
  : OdometryBase<TConfig, types::RegistrationConfig, types::RegistrationDebug>(config, debug)
  {
    // Additional initialization
#ifdef __CUDACC__
    this->setup_cuda_device();
#endif
  }
  /**
   * @brief Set the configuration of the registration handler from the param manager
   * @param [in] pmg                Param manager
   */
  void set_config(tam::pmg::ParamReferenceManager * pmg) override
  {
    // clang-format off
    pmg->declare_parameter("registration.solver_type", &this->config_.solver_type, "GaussNewton", tam::pmg::ParameterType::STRING, "Solver: GaussNewton or LevenbergMarquardt");  // NOLINT
    pmg->declare_parameter("registration.max_iter", &this->config_.max_iter, static_cast<int64_t>(500), tam::pmg::ParameterType::INTEGER, "Max solver iterations");  // NOLINT
    pmg->declare_parameter("registration.max_inner_iter", &this->config_.max_inner_iter, static_cast<int64_t>(20), tam::pmg::ParameterType::INTEGER, "Max inner iterations per LM step");  // NOLINT
    pmg->declare_parameter("registration.max_time", &this->config_.max_time, 150.0, tam::pmg::ParameterType::DOUBLE, "Max registration time (ms)");  // NOLINT
    pmg->declare_parameter("registration.convergence_criterion", &this->config_.convergence_criterion, 5.0e-3, tam::pmg::ParameterType::DOUBLE, "Convergence threshold on the update step norm");  // NOLINT
    pmg->declare_parameter("registration.damping_factor", &this->config_.damping_factor, 0.0, tam::pmg::ParameterType::DOUBLE, "Initial LM damping relative to max JTJ diagonal entry");  // NOLINT
    pmg->declare_parameter("registration.damping_scale", &this->config_.damping_scale, 2.0, tam::pmg::ParameterType::DOUBLE, "LM damping increase factor on rejected steps");  // NOLINT
    // clang-format on
  }
  /**
   * @brief Register the debug variables with the logger
   * @param [in] logger             Logger
   */
  void set_logging(tam::tsl::ReferenceLogger * logger) const override
  {
    logger->log("registration/converged", &this->debug_.converged);
    logger->log("registration/registration_time", &this->debug_.registration_time);
    logger->log("registration/damping_factor", &this->debug_.damping_factor);
    logger->log("registration/num_iter", &this->debug_.num_iter);
    logger->log("registration/num_points_frame", &this->debug_.num_points_frame);
    logger->log("registration/conditional", &this->debug_.conditional);
  }

protected:
  /**
   * @brief Solve a given frame using the configured solver type
   * @param [in] frame                    Frame to register
   * @param [in] map                      Map to register to
   * @param [in] factor                   Factor to use for the registration
   * @param [in] error                    Error to use for the registration
   * @return                              Registered pose
   * @throws std::runtime_error           If the solver type is unknown
   */
  template <typename FactorType, typename ErrorType>
#ifdef __CUDACC__
  Sophus::SE3f solve(
    thrust::device_vector<types::Point<TConfig>> & frame, const MapHandler<TConfig> * map,
    FactorType & factor, ErrorType & error, const float correspondence_threshold,
    const float kernel_scale, ::cuda::stream_ref stream = {})
#else
  Sophus::SE3f solve(
    std::vector<types::Point<TConfig>> & frame, const MapHandler<TConfig> * map,
    FactorType & factor, ErrorType & error, const float correspondence_threshold,
    const float kernel_scale)
#endif
  {
    // Prepare factor and error
    factor.set_kernel_scale(kernel_scale);
    factor.set_correspondence_threshold(correspondence_threshold);
    // Current transform is identity because we apply the initial guess to the source points
    error.set_transform(Sophus::SE3f());
    error.set_correspondence_threshold(correspondence_threshold);

    // Call the solver
    Sophus::SE3f T = Sophus::SE3f();
    if (this->config_.solver_type == "GaussNewton") {
      // Use Gauss-Newton method
#ifdef __CUDACC__
      T = this->solve_gaussnewton(frame, map, factor, error, stream);
#else
      T = this->solve_gaussnewton(frame, map, factor, error);
#endif
    } else if (this->config_.solver_type == "LevenbergMarquardt") {
      // Use Levenberg-Marquardt method
#ifdef __CUDACC__
      T = this->solve_levenbergmarquardt(frame, map, factor, error, stream);
#else
      T = this->solve_levenbergmarquardt(frame, map, factor, error);
#endif
    } else {
      throw std::runtime_error(
        "Unknown solver type: " + this->config_.solver_type +
        ". Supported types are GaussNewton and LevenbergMarquardt.");
    }
    return T;
  }
  /**
   * @brief Solve a given frame using Gauss-Newton method
   * @param [in] frame                    Frame to register
   * @param [in] map                      Map to register to
   * @param [in] factor                   Factor to use for the registration
   * @param [in] error                    Error to use for the registration
   * @return                              Registered pose
   */
  template <typename FactorType, typename ErrorType>
#ifdef __CUDACC__
  Sophus::SE3f solve_gaussnewton(
    thrust::device_vector<types::Point<TConfig>> & frame, const MapHandler<TConfig> * map,
    FactorType & factor, [[maybe_unused]] ErrorType & error, ::cuda::stream_ref stream = {})
#else
  Sophus::SE3f solve_gaussnewton(
    std::vector<types::Point<TConfig>> & frame, const MapHandler<TConfig> * map,
    FactorType & factor, [[maybe_unused]] ErrorType & error)
#endif
  {
    // Start timer
    auto start = std::chrono::high_resolution_clock::now();

    // Initialize variables
    Sophus::SE3f T_icp = Sophus::SE3f();
    types::SolverStatus solver_status = types::SolverStatus::NOT_CONVERGED;
    std::int64_t iter = 0;
    double time = 0.0;
    for (std::int64_t j = 0; j < this->config_.max_iter; ++j) {
      // Compute correspondences
#ifdef __CUDACC__
      map->search_closest_neighbor(frame, this->correspondences_device_, 1, stream);
#else
      std::vector<types::Correspondence<TConfig>> correspondences =
        utils::get_correspondences(frame, map, factor.get_correspondence_threshold());
#endif
#ifdef __CUDACC__
      if (this->correspondences_device_.empty()) break;
#else
      if (correspondences.empty()) break;
#endif

        // Build linear system
#ifdef __CUDACC__
      const types::LinearSystem & ls = cuda::utils::build_linear_system(
        this->correspondences_device_, this->ls_final_d_, factor, stream);
#else
      const types::LinearSystem & ls = utils::build_linear_system(correspondences, factor);
#endif
      // Solve linear system
      const Eigen::Matrix<float, 6, 1> dx =
        (ls.JTJ +
         static_cast<float>(this->config_.damping_factor) * Eigen::Matrix<float, 6, 6>::Identity())
          .ldlt()
          .solve(-ls.JTr);
      const Sophus::SE3f estimation = Sophus::SE3f::exp(dx);
      // Transform points with new estimate
#ifdef __CUDACC__
      cuda::utils::transform_points(estimation, frame, stream);
#else
      utils::transform_points(estimation, frame);
#endif
      // Update transformation
      T_icp = estimation * T_icp;
      iter = j;
      // Check solver status
      solver_status = this->solver_state(dx);
      // Compute elapsed time
      // clang-format off
      time = std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::high_resolution_clock::now() - start).count() * 1.0e-3;  // NOLINT
      // clang-format on
      // Check termination criteria
      if (this->termination_criteria(solver_status, iter, time)) break;
    }
    // Update debug information
    this->debug_.converged = solver_status == types::SolverStatus::CONVERGED;
    this->debug_.registration_time = time;
    this->debug_.damping_factor = static_cast<float>(this->config_.damping_factor);
    this->debug_.num_iter = iter;
    this->debug_.num_points_frame = frame.size();
    return T_icp;
  }
  /**
   * @brief Solve a given frame using Levenberg-Marquardt method
   * @param [in] frame                    Frame to register
   * @param [in] map                      Map to register to
   * @param [in] factor                   Factor to use for the registration
   * @param [in] error                    Error to use for the registration
   * @return                              Registered pose
   */
  template <typename FactorType, typename ErrorType>
#ifdef __CUDACC__
  Sophus::SE3f solve_levenbergmarquardt(
    thrust::device_vector<types::Point<TConfig>> & frame, const MapHandler<TConfig> * map,
    FactorType & factor, ErrorType & error, ::cuda::stream_ref stream = {})
#else
  Sophus::SE3f solve_levenbergmarquardt(
    std::vector<types::Point<TConfig>> & frame, const MapHandler<TConfig> * map,
    FactorType & factor, ErrorType & error)
#endif
  {
    // Start timer
    auto start = std::chrono::high_resolution_clock::now();

    // Initialize variables
    Sophus::SE3f T_icp = Sophus::SE3f();
    types::SolverStatus solver_status = types::SolverStatus::NOT_CONVERGED;
    std::int64_t iter = 0;
    std::int64_t inner_iter = 0;
    float current_error = 0.0f;
    // Initial lambda for Levenberg-Marquardt -> overwrite before first use
    float current_lambda = 0.0f;
    double time = 0.0;
    // Outer loop
    for (std::int64_t j = 0; j < this->config_.max_iter; ++j) {
      // Compute correspondences
#ifdef __CUDACC__
      map->search_closest_neighbor(frame, this->correspondences_device_, 1, stream);
#else
      std::vector<types::Correspondence<TConfig>> correspondences =
        utils::get_correspondences(frame, map, factor.get_correspondence_threshold());
#endif
#ifdef __CUDACC__
      if (this->correspondences_device_.empty()) break;
#else
      if (correspondences.empty()) break;
#endif
      // Reset error transform to identity for the current iteration
      error.set_transform(Sophus::SE3f());
        // Build linear system and compute current error
#ifdef __CUDACC__
      const types::LinearSystem & ls = cuda::utils::build_linear_system(
        this->correspondences_device_, this->ls_final_d_, factor, stream);
      current_error =
        cuda::utils::compute_error(this->correspondences_device_, this->error_d_, error, stream);
#else
      const types::LinearSystem & ls = utils::build_linear_system(correspondences, factor);
      current_error = utils::compute_error(correspondences, error);
#endif

      // Init Levenberg-Marquardt parameters
      Sophus::SE3f delta = Sophus::SE3f();
      Sophus::SE3f delta_lm = Sophus::SE3f();
      // Set initial lambda for first iteration
      if (j == 0) {
        current_lambda = std::max(
          static_cast<float>(this->config_.damping_factor) *
            ls.JTJ.diagonal().array().abs().maxCoeff(),
          1.0e-12f);
      }
      // Start Levenberg-Marquardt inner loop
      bool success_lm = false;
      float nu = static_cast<float>(this->config_.damping_scale);
      for (std::int64_t lm_iter = 0; lm_iter < this->config_.max_inner_iter; ++lm_iter) {
        // Solve linear system
        const Eigen::Matrix<float, 6, 1> candidate_dx =
          (ls.JTJ + current_lambda * Eigen::Matrix<float, 6, 6>::Identity())
            .ldlt()
            .solve(-ls.JTr);  // NOLINT
        delta_lm = Sophus::SE3f::exp(candidate_dx);
        // Set the transformation to the factor
        error.set_transform(delta_lm);
        // Compute the error for the new transform
#ifdef __CUDACC__
        float candidate_error =
          cuda::utils::compute_error(this->correspondences_device_, this->error_d_, error, stream);
#else
        float candidate_error = utils::compute_error(correspondences, error);
#endif
        float rho = (current_error - candidate_error) /
                    (candidate_dx.dot(current_lambda * candidate_dx - ls.JTr));

        // Update debug information
        inner_iter = lm_iter;
        types::SolverStatus solver_status_candidate = this->solver_state(candidate_dx);
        // Termination criteria inner loop
        if (rho < 0.0f) {
          if (solver_status_candidate == types::SolverStatus::CONVERGED) {
            solver_status = solver_status_candidate;
            success_lm = true;
            break;
          }
          // Increase the damping factor otherwise
          // (always ensure that lambda can never become nummerically unstable)
          current_lambda = std::max(nu * current_lambda, 1.0e-12f);
          nu *= 2.0f;
          // Jump to the next iteration of the inner loop
          continue;
        }
        // Update outer loop variables if rho > 0
        T_icp = delta_lm * T_icp;                 // Update complete transformation
        delta = delta_lm;                         // Update delta to transform points
        solver_status = solver_status_candidate;  // Update solver status
        current_lambda = std::max(
          current_lambda * std::max(1.0f / 3.0f, 1.0f - std::pow(2.0f * rho - 1.0f, 3.0f)),
          1.0e-12f);
        success_lm = true;
        break;  // Break the inner loop if we found a valid solution
      }
      // Break outer loop if inner loop did not succeed
      if (!success_lm) break;
        // Transform points with new estimate
#ifdef __CUDACC__
      cuda::utils::transform_points(delta, frame, stream);
#else
      utils::transform_points(delta, frame);
#endif
      // Termination criteria
      iter = j;
      // Compute elapsed time
      // clang-format off
      time = std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::high_resolution_clock::now() - start).count() * 1.0e-3;  // NOLINT
      // clang-format on
      if (this->termination_criteria(solver_status, iter, time)) break;
    }
    // Update debug information
    this->debug_.converged = solver_status == types::SolverStatus::CONVERGED;
    this->debug_.registration_time = time;
    this->debug_.damping_factor = current_lambda;
    this->debug_.conditional["error"] = static_cast<double>(current_error);
    this->debug_.num_iter = iter;
    this->debug_.conditional["num_inner_iter"] = static_cast<std::int64_t>(inner_iter);
    this->debug_.num_points_frame = frame.size();
    return T_icp;
  }
  /**
   * @brief Check solver convergence based on translational and rotational criterion
   * @param[in] dx                       Update vector
   * @return                             True, if criteria fullfilled
   */
  types::SolverStatus solver_state(const Eigen::Matrix<float, 6, 1> & dx) const
  {
    const double dx_norm = dx.norm();
    // Check if an actual solution was computed
    // As the check against the correspondence threshold is done in the linear system build,
    // it could happen that there is no valid correspondence found, thus the linear system
    // consists only of 0 and dx_norm = 0
    if (dx_norm <= 1.0e-12) return types::SolverStatus::INVALID;
    // Check convergence
    if (dx_norm < this->config_.convergence_criterion) {
      return types::SolverStatus::CONVERGED;
    } else {
      return types::SolverStatus::NOT_CONVERGED;
    }
  }
  /**
   * @brief Check optimization termination criteria
   * @param [in] status                  Current solver status
   * @param [in] iter                    Current iteration number
   * @param [in] time                    Elapsed time in milliseconds
   * @return                             True if the optimization should terminate
   */
  bool termination_criteria(
    const types::SolverStatus & status, const std::int64_t iter, const double time) const
  {
    // clang-format off
    if (status == types::SolverStatus::CONVERGED || status == types::SolverStatus::INVALID) return true;  // NOLINT
    // Check if the maximum number of iterations is reached
    if (iter >= this->config_.max_iter) return true;
    // Check if the maximum time is exceeded
    if (time >= this->config_.max_time) return true;
    // If none of the criteria are met, continue optimization
    return false;
    // clang-format on
  }

protected:
  // Map handler for the frame
  std::unique_ptr<MapHandler<TConfig>> frame_map_{nullptr};
// Cuda specific code and members
#ifdef __CUDACC__
protected:
  /**
   * @brief Setup CUDA device
   * @param [in] device_id               Device ID to use
   */
  void __host__ setup_cuda_device(int device_id = 0)
  {
    // Set the device
    this->num_multiprocessors_ = cuda::utils::set_device(device_id).multiProcessorCount;
    // Preallocate space on the GPU
    cuda::utils::allocate_vector(correspondences_device_, 50000);
    cuda::utils::allocate_vector(ls_final_d_, cuda::LS_SIZE);
    cudaMalloc(&error_d_, sizeof(float));
    cudaMemset(error_d_, 0, sizeof(float));
    // Synchronize to ensure the context is initialized
    cudaDeviceSynchronize();
  }

protected:
  // Number of multiprocessors on the device
  size_t num_multiprocessors_{0};
  // Preallocated device vectors
  mutable thrust::device_vector<types::Correspondence<TConfig>> correspondences_device_;
  mutable thrust::device_vector<float> ls_final_d_;
  mutable float * error_d_;
#endif
};
}  // namespace tam::core::state
