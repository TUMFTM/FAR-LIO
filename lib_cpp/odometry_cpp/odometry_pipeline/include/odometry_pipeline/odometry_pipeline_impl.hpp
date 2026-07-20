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

// Include interface modules -> include here so headers are not public
#include "covariance_handler/constant_covariance.hpp"
#include "diagnostic_handler/diagnostic_handler_base.hpp"
#include "distortion_handler/polynom_undistortion.hpp"
#include "map_handler/voxel_hash_map.hpp"
#include "model_handler/constant_velocity.hpp"
#include "model_handler/external_guess.hpp"
#include "preprocessing_handler/lidar_preprocessing.hpp"
#include "registration_handler/gicp.hpp"
#include "registration_handler/icp.hpp"
#include "threshold_handler/adaptive_threshold.hpp"
#include "threshold_handler/fixed_threshold.hpp"
#include "velocity_handler/derivative.hpp"

#ifdef __CUDACC__
#include "distortion_handler/polynom_undistortion.cuh"
#include "map_handler/voxel_hash_map.cuh"
#include "preprocessing_handler/lidar_preprocessing.cuh"
#include "registration_handler/gicp.cuh"
#include "registration_handler/icp.cuh"
#endif

#include <algorithm>
#include <iostream>
#include <memory>
#include <vector>

#include "odometry_pipeline/odometry_pipeline.hpp"

namespace tam::core::state {
/**
 * @brief Constructor for param manager and logger
 */
template <typename TConfig>
std::unique_ptr<OdometryPipeline<TConfig>> OdometryPipeline<TConfig>::from_config(
  tam::pmg::ParamReferenceManager* pmg, tam::tsl::ReferenceLogger* logger)
{
  std::unique_ptr<OdometryPipeline<TConfig>> op =
    std::unique_ptr<OdometryPipeline<TConfig>>(new OdometryPipeline<TConfig>(pmg, logger));
  return op;
}

/**
 * @brief Constructor for config and debug objects
 */
template <typename TConfig>
std::unique_ptr<OdometryPipeline<TConfig>> OdometryPipeline<TConfig>::from_config(
  const types::PipelineConfig& config, const types::PipelineDebug& debug)
{
  std::unique_ptr<OdometryPipeline<TConfig>> op =
    std::unique_ptr<OdometryPipeline<TConfig>>(new OdometryPipeline<TConfig>(config, debug));
  return op;
}

// Interface functions
/**
 * @brief Register a frame to the map
 * @param [in] frame                    Frame to register
 * @param [in] frame_stamp              Timestamp of the frame (ns)
 * @return                              Odometry output with pose, tangent, covariances, and status
 */
template <typename TConfig>
types::Odometry OdometryPipeline<TConfig>::register_frame(
  const std::vector<types::Point<TConfig>>& frame, const std::uint64_t frame_stamp)
{
  // If an async map update staged a replacement, use it
  this->map_->consume_map_switch();

  // Get initial guess
  types::PoseStamped init_guess = this->model_->get_initial_guess(frame_stamp);
  // Take z-position from buffered pose if it was valid
  if (this->odometry_.status.level == types::DiagnosticLevel::OK && this->config_.buffer_z) {
    init_guess.pose.translation().z() = this->odometry_.pose.pose.translation().z();
  }

  // Catch empty frame
  // Copying an empty vector to the GPU leads to an invalid device ordinal error
  if (frame.empty()) {
    types::Odometry odom{};
    odom.stamp = frame_stamp;
    odom.pose.pose = init_guess.pose;
    odom.status.level = types::DiagnosticLevel::ERROR;
    odom.status.message = "empty cloud";
    return odom;
  }

  // Preprocessing
  const double map_resolution = this->map_->get_resolution();

  // Allocate frame registration and map
#ifdef __CUDACC__
  nvtxRangePush("setup_frame");
  // Load the data to the GPU and double downsample frame
  frame_.resize(frame.size());
  cudaMemcpy(thrust::raw_pointer_cast(frame_.data()), frame.data(), frame.size() * sizeof(types::Point<TConfig>),
    cudaMemcpyHostToDevice);
  // Undistort frame if configured
  if (this->config_.undistort) {
    // Perform in-place undistortion
    this->distortion_->undistort(frame_, frame_stamp, this->stream_);
  }
  // Preprocess frame if configured
  if (this->config_.preprocess) {
    if (!this->preprocessing_->preprocess(frame_, this->stream_)) {
      types::Odometry odom{};
      odom.stamp = frame_stamp;
      odom.pose.pose = init_guess.pose;
      odom.status.level = types::DiagnosticLevel::ERROR;
      odom.status.message = "Empty cloud after preprocessing!";
      return odom;
    }
  }
  // Start timer for downsample time
  auto start = std::chrono::high_resolution_clock::now();
  // Downsample frame
  if (this->config_.downsample) {
    cuda::voxel_doubledownsample_device(
      frame_, frame_map_, frame_registration_, this->map_->get_config().voxel_size, 50000, this->stream_);
  } else {
    // Downsample the point cloud to match the map resulution to ensure a faster point insertion
    // This is necessary restict the number of threads that want to insert into the same voxel
    cuda::voxel_downsample(frame_, frame_map_, map_resolution, 50000, this->stream_);
    frame_registration_.resize(frame_map_.size());
    thrust::copy(
      thrust::cuda::par.on(this->stream_), frame_map_.begin(), frame_map_.end(), frame_registration_.begin());
  }
  nvtxRangePop();
#else
  std::vector<types::Point<TConfig>> frame_preprocessed = frame;
  // Undistort frame if configured
  if (this->config_.undistort) {
    this->distortion_->undistort(frame_preprocessed, frame_stamp);
  }
  if (this->config_.preprocess) {
    if (!this->preprocessing_->preprocess(frame_preprocessed)) {
      types::Odometry odom{};
      odom.stamp = frame_stamp;
      odom.pose.pose = init_guess.pose;
      odom.status.level = types::DiagnosticLevel::ERROR;
      odom.status.message = "Empty cloud after preprocessing!";
      return odom;
    }
  }
  // Start timer for downsample
  auto start = std::chrono::high_resolution_clock::now();
  // Downsample frame
  if (this->config_.downsample) {
    std::tie(frame_registration_, frame_map_) =
      voxel_doubledownsample(frame_preprocessed, this->map_->get_config().voxel_size);
  } else {
    // Downsample the point cloud to match the map resulution to ensure a faster point insertion
    frame_registration_ = voxel_downsample(frame_preprocessed, map_resolution);
    frame_map_ = frame_registration_;
  }
#endif

  // Update debug values for downsampling time
  this->debug_.num_points_frame = frame.size();
  // clang-format off
  this->debug_.downsample_time = std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::high_resolution_clock::now() - start).count() * 1.0e-3;  // NOLINT
  // clang-format on

  // Get correspondence threshold
  const double sigma = this->threshold_->get_threshold();

  // Register frame
#ifdef __CUDACC__
  const Sophus::SE3f pose_registered = this->registration_->register_frame(
    frame_registration_, this->map_.get(), init_guess.pose, 3.0 * sigma, sigma / 3.0, this->stream_);
#else
  const Sophus::SE3f pose_registered = this->registration_->register_frame(
    frame_registration_, this->map_.get(), init_guess.pose, 3.0 * sigma, sigma / 3.0);
#endif
  // Create a Pose with timestamp from the registered pose
  types::PoseStamped pose_registered_stamped;
  pose_registered_stamped.stamp = frame_stamp;
  pose_registered_stamped.pose = pose_registered;

  // Compute the tangent (velocity) using the velocity handler
  types::TangentStamped tangent{};
  tangent = this->velocity_->get_tangent({}, pose_registered_stamped);

  // Get diagnostic status
  const types::DiagnosticStatus diag_status = this->diagnostic_->get_diagnostic_status(
    pose_registered_stamped, tangent, init_guess, this->registration_->get_registration_status());

  // Set model deviation
  this->threshold_->set_model_deviation(init_guess.pose, pose_registered);
  // Set pose to model handler to compute initial guess for next frame if initial guess
  // is not captured from EXTERNALGUESS
  if constexpr (TConfig::MODEL != types::ModelType::EXTERNALGUESS) {
    this->model_->set_pose(pose_registered_stamped, true);
  }
  // Update map if configured and registration was successful
  if (this->config_.update_map) {
    std::variant<int16_t, types::AdaptiveMapDensity> map_density = TConfig::MAX_POINTS_PER_VOXEL;
#ifdef __CUDACC__
    if constexpr (types::FRAMEMAP<TConfig>) {
      const types::AdaptiveMapDensity adaptive_density =
        this->registration_->get_frame_map_density(20, pose_registered.translation(), this->stream_);
      if (adaptive_density.valid) map_density = adaptive_density;
    }
    this->map_->update_points(frame_map_, pose_registered, map_density, 1, TConfig::NUM_NEIGHBORS, this->stream_);
#else
    if constexpr (types::FRAMEMAP<TConfig>) {
      const types::AdaptiveMapDensity adaptive_density =
        this->registration_->get_frame_map_density(20, pose_registered.translation());
      if (adaptive_density.valid) map_density = adaptive_density;
    }
    this->map_->update_points(frame_map_, pose_registered, map_density, 1, TConfig::NUM_NEIGHBORS);
#endif
  }

  // Build Odometry output
  types::Odometry odom;
  odom.stamp = frame_stamp;
  odom.pose.pose = pose_registered;
  odom.tangent.tangent = tangent.tangent;
  odom.status = diag_status;

  // Get correspondences of final registration result for covariance estimation
  std::vector<types::Correspondence<TConfig>> correspondences{};
  if constexpr (TConfig::COVARIANCE != types::CovarianceType::CONSTANT) {
#ifdef __CUDACC__
    const auto& correspondences = this->registration_->get_correspondences(
      frame_registration_, this->map_.get(), pose_registered, 3.0 * sigma, this->stream_);
    // Sync stream
    cudaStreamSynchronize(this->stream_);
#else
    const auto& correspondences =
      this->registration_->get_correspondences(frame_registration_, this->map_.get(), pose_registered, 3.0 * sigma);
#endif
  }
  // Get covariances
  odom.pose.covariance = this->covariance_->get_pose_covariance(pose_registered, correspondences, sigma / 3.0);
  odom.tangent.covariance = this->covariance_->get_tangent_covariance();

  // Buffer odometry
  this->odometry_ = odom;

#ifdef __CUDACC__
  // Copy downsampled frame to host when in debug mode
  if (this->config_.debug_mode) {
    this->frame_downsampled_.resize(frame_registration_.size());
    thrust::copy(frame_registration_.begin(), frame_registration_.end(), this->frame_downsampled_.begin());
  }
#else
  this->frame_downsampled_ = frame_registration_;
#endif
  // Log the whole pipeline time
  // clang-format off
  this->debug_.pipeline_time = std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::high_resolution_clock::now() - start).count() * 1.0e-3;  // NOLINT
  // clang-format on
  return odom;
}

/**
 * @brief Init threading on CPU
 * -> needs to be done after param overrides
 */
template <typename TConfig>
void OdometryPipeline<TConfig>::init()
{
  this->distortion_->init(this->config_.num_threads);
  this->map_->init(this->config_.num_threads, this->config_.update_map);
  this->registration_->init(this->config_.num_threads);
  this->preprocessing_->init();
  this->threshold_->init();
  if constexpr (types::FRAMEMAP<TConfig>) {
    this->registration_->init_frame_map(this->map_->get_config());
  }
}

/**
 * @brief Cleans up all allocated memory and destroys the stream
 */
template <typename TConfig>
void OdometryPipeline<TConfig>::free()
{
  frame_registration_.clear();
  frame_registration_.shrink_to_fit();
  frame_map_.clear();
  frame_map_.shrink_to_fit();
  frame_.clear();
  frame_.shrink_to_fit();
}

/**
 * @brief Init model
 */
template <typename TConfig>
void OdometryPipeline<TConfig>::init_model_config()
{
  this->model_->init_model_config();
}

/**
 * @brief Set the status of the input pointcloud
 * @param [in] status                   Status to set
 */
template <typename TConfig>
void OdometryPipeline<TConfig>::set_input_status(const types::DiagnosticStatus& status)
{
  this->diagnostic_->set_input_status(status);
}

/**
 * @brief Set the pose from an external source
 * @param [in] pose                   Pose to set
 * @param [in] valid                  Whether the given pose is valid
 */
template <typename TConfig>
void OdometryPipeline<TConfig>::set_pose_model(const types::PoseStamped& pose, const bool valid)
{
  model_->set_pose(pose, valid);
}

/**
 * @brief Set a pose from an external source (e.g. sampled from tf) used for deskewing
 * @param [in] pose                   Pose to set
 */
template <typename TConfig>
void OdometryPipeline<TConfig>::set_pose_undistortion(const types::PoseStamped& pose)
{
  distortion_->set_pose(pose);
}

/**
 * @brief Get the current odometry estimate
 * @return                        Current odometry estimate
 */
template <typename TConfig>
types::Odometry OdometryPipeline<TConfig>::get_odometry() const
{
  return this->odometry_;
}

/**
 * @brief Access frame used for registration
 * @return                        Frame used for registration (after downsampling)
 */
template <typename TConfig>
std::vector<types::Point<TConfig>> OdometryPipeline<TConfig>::get_frame() const
{
#ifdef __CUDACC__
  // throw a error here
  if (!this->config_.debug_mode) {
    throw std::runtime_error("get_frame() cannot be called in CUDA code when debug mode is disabled");
  }
#endif
  return this->frame_downsampled_;
}

/**
 * @brief Add points to map
 * @param [in] points                   Points to add
 * @param [in] map_density              Map density to use for adding points (either max points per
 * voxel or adaptive map density)
 * @param [in] adjacent_voxels          Adjacent voxels to search in for covariances
 * @param [in] num_neighbors            Number of neighbors to consider for covariances
 * @param [in] use_active_map           Flag to indicate if the active or inactive map should be
 * used for adding points
 */
template <typename TConfig>
void OdometryPipeline<TConfig>::add_points(const std::vector<types::Point<tam::core::state::types::Point_XYZ>>& points,
  const std::variant<int16_t, types::AdaptiveMapDensity>& map_density, const int adjacent_voxels,
  const int num_neighbors, const bool use_active_map)
{
#ifdef __CUDACC__
  // Add points to map
  this->map_->add_points(points, map_density, adjacent_voxels, num_neighbors, use_active_map, this->stream_);
  cudaStreamSynchronize(this->stream_);
#else
  this->map_->add_points(points, map_density, adjacent_voxels, num_neighbors, use_active_map);
#endif
}

/**
 * @brief Request a switch between active and inactive map
 */
template <typename TConfig>
void OdometryPipeline<TConfig>::request_map_switch()
{
  this->map_->request_map_switch();
}

/**
 * @brief Expose the map handler's async-update mutex so the node can hold it
 *        across the full async add_points(inactive) + request_async_switch
 *        sequence.
 */
template <typename TConfig>
std::mutex& OdometryPipeline<TConfig>::get_map_mutex()
{
  return this->map_->get_mutex();
}

/**
 * @brief Get the map resolution
 * @return                        Map resolution
 */
template <typename TConfig>
double OdometryPipeline<TConfig>::get_map_resolution() const
{
  return this->map_->get_resolution();
}

/**
 * @brief Return the points in the map as vector
 * @return                        Vector of points in the map
 */
template <typename TConfig>
std::vector<types::Point<TConfig>> OdometryPipeline<TConfig>::get_map() const
{
#ifdef __CUDACC__
  // Get positions and normals
  thrust::device_vector<types::Point<TConfig>> device_points = map_->get_cloud();
  std::vector<types::Point<TConfig>> host_points(device_points.size());
  // Copy points from device to host
  thrust::copy(device_points.begin(), device_points.end(), host_points.begin());
  return host_points;
#else
  return map_->get_cloud();
#endif
}

/**
 * @brief Get the current registration status
 */
template <typename TConfig>
types::RegistrationStatus OdometryPipeline<TConfig>::get_registration_status() const
{
  return this->registration_->get_registration_status();
}

/**
 * @brief Access module configs
 */
// clang-format off
template <typename TConfig>
types::CovarianceConfig & OdometryPipeline<TConfig>::get_covariance_config() { return covariance_->get_config(); }  // NOLINT
template <typename TConfig>
types::DiagnosticConfig & OdometryPipeline<TConfig>::get_diagnostic_config() { return diagnostic_->get_config(); }  // NOLINT
template <typename TConfig>
types::DistortionConfig & OdometryPipeline<TConfig>::get_distortion_config() { return distortion_->get_config(); }  // NOLINT
template <typename TConfig>
types::MapConfig & OdometryPipeline<TConfig>::get_map_config() { return map_->get_config(); }  // NOLINT
template <typename TConfig>
types::ModelConfig & OdometryPipeline<TConfig>::get_model_config() { return model_->get_config(); }  // NOLINT
template <typename TConfig>
types::PreprocessingConfig & OdometryPipeline<TConfig>::get_preprocessing_config() { return preprocessing_->get_config(); }  // NOLINT
template <typename TConfig>
types::RegistrationConfig & OdometryPipeline<TConfig>::get_registration_config() { return registration_->get_config(); }  // NOLINT
template <typename TConfig>
types::ThresholdConfig & OdometryPipeline<TConfig>::get_threshold_config() { return threshold_->get_config(); }  // NOLINT
template <typename TConfig>
types::VelocityConfig & OdometryPipeline<TConfig>::get_velocity_config() { return velocity_->get_config(); }  // NOLINT
/**
 * @brief Access module debugs
 */
template <typename TConfig>
types::CovarianceDebug OdometryPipeline<TConfig>::get_covariance_debug() const { return covariance_->get_debug(); } // NOLINT
template <typename TConfig>
types::DiagnosticDebug OdometryPipeline<TConfig>::get_diagnostic_debug() const { return diagnostic_->get_debug(); }  // NOLINT
template <typename TConfig>
types::DistortionDebug OdometryPipeline<TConfig>::get_distortion_debug() const { return distortion_->get_debug(); } // NOLINT
template <typename TConfig>
types::MapDebug OdometryPipeline<TConfig>::get_map_debug() const { return map_->get_debug(); }  // NOLINT
template <typename TConfig>
types::ModelDebug OdometryPipeline<TConfig>::get_model_debug() const { return model_->get_debug(); }  // NOLINT
template <typename TConfig>
types::PreprocessingDebug OdometryPipeline<TConfig>::get_preprocessing_debug() const { return preprocessing_->get_debug(); } // NOLINT
template <typename TConfig>
types::RegistrationDebug OdometryPipeline<TConfig>::get_registration_debug() const { return registration_->get_debug(); }  // NOLINT
template <typename TConfig>
types::ThresholdDebug OdometryPipeline<TConfig>::get_threshold_debug() const { return threshold_->get_debug(); } // NOLINT
template <typename TConfig>
types::VelocityDebug OdometryPipeline<TConfig>::get_velocity_debug() const { return velocity_->get_debug(); }  // NOLINT

// clang-format on
// Inherit constructor from OdometryBase for param manager and logger

template <typename TConfig>
OdometryPipeline<TConfig>::OdometryPipeline(tam::pmg::ParamReferenceManager* pmg, tam::tsl::ReferenceLogger* logger)
    : OdometryBase<TConfig, types::PipelineConfig, types::PipelineDebug>(pmg, logger)
{
  // Set pipelie parameters
  this->set_config(pmg);
  this->set_logging(logger);

  // Initialize modules
  // -> add a new line here if you add a new module version
  // clang-format off
    // CovarianceHandler
    INIT_MODULE(COVARIANCE, CovarianceType::CONSTANT, covariance_, ConstantCovariance, pmg, logger);  // NOLINT
    // DiagnosticHandler
    INIT_MODULE(DIAGNOSTIC, DiagnosticType::BASE, diagnostic_, DiagnosticHandler, pmg, logger);
    // DistortionHandler
    INIT_MODULE(DISTORTION, DistortionType::POLYNOM, distortion_, PolynomUndistortion, pmg, logger);  // NOLINT
    // MapHandler
    INIT_MODULE(MAP, MapType::VOXELHASHMAP, map_, VoxelHashMap, pmg, logger);
    // ModelHandler
    INIT_MODULE(MODEL, ModelType::CONSTANTVELOCITY, model_, ConstantVelocity, pmg, logger);
    INIT_MODULE(MODEL, ModelType::EXTERNALGUESS, model_, ExternalGuess, pmg, logger);
    // PreprocessingHandler
    INIT_MODULE(PREPROCESSING, PreprocessingType::LIDAR, preprocessing_, LidarPreprocessing, pmg, logger);  // NOLINT
    // RegistrationHandler
    INIT_MODULE(REGISTRATION, RegistrationType::ICP, registration_, ICP, pmg, logger);
    INIT_MODULE(REGISTRATION, RegistrationType::GICP, registration_, GICP, pmg, logger);
    // ThresholdHandler
    INIT_MODULE(THRESHOLD, ThresholdType::FIXEDTHRESHOLD, threshold_, FixedThreshold, pmg, logger);  // NOLINT
    INIT_MODULE(THRESHOLD, ThresholdType::ADAPTIVETHRESHOLD, threshold_, AdaptiveThreshold, pmg, logger);  // NOLINT
    // VelocityHandler
    INIT_MODULE(VELOCITY, VelocityType::DERIVATIVE, velocity_, Derivative, pmg, logger);
    // CUDA modules
#ifdef __CUDACC__
    INIT_MODULE(DISTORTION, DistortionType::CUDA_POLYNOM, distortion_, cuda::PolynomUndistortion, pmg, logger);  // NOLINT
    INIT_MODULE(MAP, MapType::CUDA_VOXELHASHMAP, map_, cuda::VoxelHashMap, pmg, logger);
    INIT_MODULE(PREPROCESSING, PreprocessingType::CUDA_LIDAR, preprocessing_, cuda::LidarPreprocessing, pmg, logger);  // NOLINT
    INIT_MODULE(REGISTRATION, RegistrationType::CUDA_ICP, registration_, cuda::ICP, pmg, logger);
    INIT_MODULE(REGISTRATION, RegistrationType::CUDA_GICP, registration_, cuda::GICP, pmg, logger);
    this->init_cuda();
#else
    // preallocate memory of the input point clouds
    frame_registration_.reserve(30000);
    frame_map_.reserve(30000);
    frame_.reserve(300000);
#endif
  // clang-format on
  // Additional initialization
  std::cout << "\033[1;36mInitialized odometry pipeline!\033[0m" << std::endl;
}

// Inherit constructor from OdometryBase for config and debug object
template <typename TConfig>
OdometryPipeline<TConfig>::OdometryPipeline(const types::PipelineConfig& config, const types::PipelineDebug& debug)
    : OdometryBase<TConfig, types::PipelineConfig, types::PipelineDebug>(config, debug)
{
  // Initialize modules
  // -> add a new line here if you add a new module version
  // clang-format off
    // CovarianceHandler
    INIT_MODULE(COVARIANCE, CovarianceType::CONSTANT, covariance_, ConstantCovariance, types::CovarianceConfig{}, types::CovarianceDebug{});  // NOLINT
    // DiagnosticHandler
    INIT_MODULE(DIAGNOSTIC, DiagnosticType::BASE, diagnostic_, DiagnosticHandler, types::DiagnosticConfig{}, types::DiagnosticDebug{});  // NOLINT
    // DistortionHandler
    INIT_MODULE(DISTORTION, DistortionType::POLYNOM, distortion_, PolynomUndistortion, types::DistortionConfig{}, types::DistortionDebug{});  // NOLINT
    // MapHandler
    INIT_MODULE(MAP, MapType::VOXELHASHMAP, map_, VoxelHashMap, types::MapConfig{}, types::MapDebug{});  // NOLINT
    // ModelHandler
    INIT_MODULE(MODEL, ModelType::CONSTANTVELOCITY, model_, ConstantVelocity, types::ModelConfig{}, types::ModelDebug{});  // NOLINT
    INIT_MODULE(MODEL, ModelType::EXTERNALGUESS, model_, ExternalGuess, types::ModelConfig{}, types::ModelDebug{});  // NOLINT
    // PreprocessingHandler
    INIT_MODULE(PREPROCESSING, PreprocessingType::LIDAR, preprocessing_, LidarPreprocessing, types::PreprocessingConfig{}, types::PreprocessingDebug{});  // NOLINT
    // RegistrationHandler
    INIT_MODULE(REGISTRATION, RegistrationType::ICP, registration_, ICP, types::RegistrationConfig{}, types::RegistrationDebug{});  // NOLINT
    INIT_MODULE(REGISTRATION, RegistrationType::GICP, registration_, GICP, types::RegistrationConfig{}, types::RegistrationDebug{});  // NOLINT
    // ThresholdHandler
    INIT_MODULE(THRESHOLD, ThresholdType::FIXEDTHRESHOLD, threshold_, FixedThreshold, types::ThresholdConfig{}, types::ThresholdDebug{});  // NOLINT
    INIT_MODULE(THRESHOLD, ThresholdType::ADAPTIVETHRESHOLD, threshold_, AdaptiveThreshold, types::ThresholdConfig{}, types::ThresholdDebug{});  // NOLINT
    // VelocityHandler
    INIT_MODULE(VELOCITY, VelocityType::DERIVATIVE, velocity_, Derivative, types::VelocityConfig{}, types::VelocityDebug{});  // NOLINT
#ifdef __CUDACC__
    INIT_MODULE(DISTORTION, DistortionType::CUDA_POLYNOM, distortion_, cuda::PolynomUndistortion, types::DistortionConfig{}, types::DistortionDebug{});  // NOLINT
    INIT_MODULE(MAP, MapType::CUDA_VOXELHASHMAP, map_, cuda::VoxelHashMap, types::MapConfig{}, types::MapDebug{});  // NOLINT
    INIT_MODULE(PREPROCESSING, PreprocessingType::CUDA_LIDAR, preprocessing_, cuda::LidarPreprocessing, types::PreprocessingConfig{}, types::PreprocessingDebug{});  // NOLINT
    INIT_MODULE(REGISTRATION, RegistrationType::CUDA_ICP, registration_, cuda::ICP, types::RegistrationConfig{}, types::RegistrationDebug{});  // NOLINT
    INIT_MODULE(REGISTRATION, RegistrationType::CUDA_GICP, registration_, cuda::GICP, types::RegistrationConfig{}, types::RegistrationDebug{});  // NOLINT
    this->init_cuda();
#else
    // preallocate memory of the input point clouds
    frame_registration_.reserve(30000);
    frame_map_.reserve(30000);
    frame_.reserve(300000);
#endif
  // clang-format on
  // Additional initialization
  std::cout << "\033[1;36mInitialized odometry pipeline!\033[0m" << std::endl;
}

/**
 * @brief Set the configuration of the pipeline from the param manager
 * @param [in] pmg                Param manager
 */
template <typename TConfig>
void OdometryPipeline<TConfig>::set_config(tam::pmg::ParamReferenceManager* pmg)
{
  // clang-format off
  pmg->declare_parameter("pipeline.update_map", &this->config_.update_map, false, tam::pmg::ParameterType::BOOL, "Update map with new points");  // NOLINT
  pmg->declare_parameter("pipeline.undistort", &this->config_.undistort, true, tam::pmg::ParameterType::BOOL, "Apply motion distortion correction");  // NOLINT
  pmg->declare_parameter("pipeline.preprocess", &this->config_.preprocess, true, tam::pmg::ParameterType::BOOL, "Preprocess point cloud");  // NOLINT
  pmg->declare_parameter("pipeline.downsample", &this->config_.downsample, true, tam::pmg::ParameterType::BOOL, "Downsample point cloud");  // NOLINT
  pmg->declare_parameter("pipeline.buffer_z", &this->config_.buffer_z, true, tam::pmg::ParameterType::BOOL, "Buffer Z position from previous pose");  // NOLINT
  pmg->declare_parameter("pipeline.debug_mode", &this->config_.debug_mode, false, tam::pmg::ParameterType::BOOL, "Enable debug mode");  // NOLINT
  pmg->declare_parameter("pipeline.num_threads", &this->config_.num_threads, static_cast<int64_t>(1), tam::pmg::ParameterType::INTEGER, "Number of TBB threads (CPU pipeline)");  // NOLINT
  // clang-format on
}

/**
 * @brief Register the debug variables with the logger
 * @param [in] logger             Logger
 */
template <typename TConfig>
void OdometryPipeline<TConfig>::set_logging(tam::tsl::ReferenceLogger* logger) const
{
  logger->log("pipeline/downsample_time", &this->debug_.downsample_time);
  logger->log("pipeline/pipeline_time", &this->debug_.pipeline_time);
  logger->log("pipeline/num_points_frame", &this->debug_.num_points_frame);
}

/**
 * @brief Define CUDA specific function to handle CUDA stream
 */
#ifdef __CUDACC__
/**
 * @brief Initialize CUDA stream
 */
template <typename TConfig>
void OdometryPipeline<TConfig>::init_cuda()
{
  // Create CUDA stream
  cudaStreamCreate(&this->stream_);

  // preallocate memory of the input point clouds
  cuda::utils::allocate_vector(frame_, 300000);
  cuda::utils::allocate_vector(frame_registration_, 30000);
  cuda::utils::allocate_vector(frame_map_, 30000);
}

// Destructor
template <typename TConfig>
OdometryPipeline<TConfig>::~OdometryPipeline()
{
  // Free the preallocated memory
  free();
#ifdef __CUDACC__
  // Destroy CUDA stream
  cudaStreamSynchronize(this->stream_);
  cudaStreamDestroy(this->stream_);
#endif
}
#endif
}  // namespace tam::core::state
