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

// CAUTION: These headers are visible to the consumer
#include <memory>
#include <mutex>
#include <vector>

#include "covariance_handler/covariance_handler_base.hpp"
#include "diagnostic_handler/diagnostic_handler_base.hpp"
#include "distortion_handler/distortion_handler_base.hpp"
#include "map_handler/map_handler_base.hpp"
#include "model_handler/model_handler_base.hpp"
#include "odometry_types/odometry_config.hpp"
#include "odometry_types/odometry_types.hpp"
#include "odometry_types/pipeline.hpp"
#include "odometry_types/point_types.hpp"
#include "preprocessing_handler/preprocessing_handler_base.hpp"
#include "registration_handler/registration_handler_base.hpp"
#include "threshold_handler/threshold_handler_base.hpp"
#include "velocity_handler/velocity_handler_base.hpp"

namespace tam::core::state {
template <typename TConfig>
class OdometryPipeline : public OdometryBase<TConfig, types::PipelineConfig, types::PipelineDebug>
{
public:
  /**
   * @brief Constructor for param manager and logger
   */
  static std::unique_ptr<OdometryPipeline<TConfig>> from_config(
    tam::pmg::ParamReferenceManager* pmg, tam::tsl::ReferenceLogger* logger);
  /**
   * @brief Constructor for config and debug objects
   */
  static std::unique_ptr<OdometryPipeline<TConfig>> from_config(
    const types::PipelineConfig& config, const types::PipelineDebug& debug);

public:
  // Interface functions
  /**
   * @brief Register a frame to the map
   * @param [in] frame                    Frame to register
   * @param [in] frame_stamp              Timestamp of the frame (ns)
   * @return                              Odometry output with pose, twist, covariances, and status
   */
  types::Odometry register_frame(const std::vector<types::Point<TConfig>>& frame, const std::uint64_t frame_stamp);
  /**
   * @brief Initialize necessary submodules in member modules
   * -> needs to be done after param overrides
   */
  void init();
  /**
   * @brief Cleans up all allocated memory and destroys the stream
   */
  void free();
  /**
   * @brief Init model form config
   */
  void init_model_config();
  /**
   * @brief Set the status of the input pointcloud
   * @param [in] status                   Status to set
   */
  void set_input_status(const types::DiagnosticStatus& status);
  /**
   * @brief Set the pose from an external source
   * @param [in] pose                   Pose to set
   * @param [in] valid                  Whether the given pose is valid
   */
  void set_pose_model(const types::PoseStamped& pose, const bool valid);
  /**
   * @brief Set a pose from an external source (e.g. sampled from tf) used for deskewing
   * @param [in] pose                   Pose to set
   */
  void set_pose_undistortion(const types::PoseStamped& pose);
  /**
   * @brief Get the current odometry estimate
   * @return                        Current odometry estimate
   */
  types::Odometry get_odometry() const;
  /**
   * @brief Access frame used for registration
   * @return                        Frame used for registration (after downsampling)
   */
  std::vector<types::Point<TConfig>> get_frame() const;
  /**
   * @brief Add points to map
   * @param [in] points                   Points to add
   * @param [in] map_density              Map density to use for adding points (either max points per voxel
   * or adaptive map density)
   * @param [in] adjacent_voxels          Adjacent voxels to search in for covariances
   * @param [in] num_neighbors            Number of neighbors to consider for covariances
   * @param [in] use_active_map           Flag to indicate if the active or inactive map should be used
   */
  void add_points(const std::vector<types::Point<tam::core::state::types::Point_XYZ>>& points,
    const std::variant<int16_t, types::AdaptiveMapDensity>& map_density = TConfig::MAX_POINTS_PER_VOXEL,
    const int adjacent_voxels = 1, const int num_neighbors = TConfig::NUM_NEIGHBORS, const bool use_active_map = true);
  /**
   * @brief Request a switch between active and inactive map
   */
  void request_map_switch();
  /**
   * @brief Get mutex for the map handler for async map updates
   */
  std::mutex& get_map_mutex();
  /**
   * @brief Get the map resolution
   * @return                        Map resolution
   */
  double get_map_resolution() const;
  /**
   * @brief Return the points in the map as vector
   * @return                        Vector of points in the map
   */
  std::vector<types::Point<TConfig>> get_map() const;
  /**
   * @brief Get the current registration status
   */
  types::RegistrationStatus get_registration_status() const;
  /**
   * @brief Access module configs
   */
  types::CovarianceConfig& get_covariance_config();
  types::DiagnosticConfig& get_diagnostic_config();
  types::DistortionConfig& get_distortion_config();
  types::MapConfig& get_map_config();
  types::ModelConfig& get_model_config();
  types::PreprocessingConfig& get_preprocessing_config();
  types::RegistrationConfig& get_registration_config();
  types::ThresholdConfig& get_threshold_config();
  types::VelocityConfig& get_velocity_config();
  /**
   * @brief Access module debugs
   */
  types::CovarianceDebug get_covariance_debug() const;
  types::DiagnosticDebug get_diagnostic_debug() const;
  types::DistortionDebug get_distortion_debug() const;
  types::MapDebug get_map_debug() const;
  types::ModelDebug get_model_debug() const;
  types::PreprocessingDebug get_preprocessing_debug() const;
  types::RegistrationDebug get_registration_debug() const;
  types::ThresholdDebug get_threshold_debug() const;
  types::VelocityDebug get_velocity_debug() const;

protected:
  // Inherit constructor from OdometryBase for param manager and logger
  OdometryPipeline(tam::pmg::ParamReferenceManager* pmg, tam::tsl::ReferenceLogger* logger);
  // Inherit constructor from OdometryBase for config and debug object
  OdometryPipeline(const types::PipelineConfig& config, const types::PipelineDebug& debug);
  /**
   * @brief Set the configuration of the pipeline from the param manager
   * @param [in] pmg                Param manager
   */
  void set_config(tam::pmg::ParamReferenceManager* pmg) override;
  /**
   * @brief Register the debug variables with the logger
   * @param [in] logger             Logger
   */
  void set_logging(tam::tsl::ReferenceLogger* logger) const override;

protected:
  // Modules
  std::unique_ptr<CovarianceHandler<TConfig>> covariance_{};
  std::unique_ptr<DiagnosticHandler<TConfig>> diagnostic_{};
  std::unique_ptr<DistortionHandler<TConfig>> distortion_{};
  std::unique_ptr<MapHandler<TConfig>> map_{};
  std::unique_ptr<ModelHandler<TConfig>> model_{};
  std::unique_ptr<PreprocessingHandler<TConfig>> preprocessing_{};
  std::unique_ptr<RegistrationHandler<TConfig>> registration_{};
  std::unique_ptr<ThresholdHandler<TConfig>> threshold_{};
  std::unique_ptr<VelocityHandler<TConfig>> velocity_{};
  // Store frame used for registration (i.e. after downsmapling)
  std::vector<types::Point<TConfig>> frame_downsampled_{};
  types::Odometry odometry_{};

// CUDA specific functions and members to handle stream
#ifdef __CUDACC__
public:
  // Destructor (needs to be public for unique_ptr)
  ~OdometryPipeline();
  // Define compiler-generated functions to follow rule of 5
  OdometryPipeline(const OdometryPipeline& other) = default;
  OdometryPipeline(OdometryPipeline&& other) = default;
  OdometryPipeline& operator=(const OdometryPipeline& other) = default;
  OdometryPipeline& operator=(OdometryPipeline&& other) = default;

protected:
  // Don't enable public construction without initialization
  OdometryPipeline() = default;
  // CUDA stream initialization
  void init_cuda();
  // CUDA stream
  cudaStream_t stream_{};
  // Member variables for the frame to allow preallocation
  thrust::device_vector<types::Point<TConfig>> frame_{};
  thrust::device_vector<types::Point<TConfig>> frame_map_{};
  thrust::device_vector<types::Point<TConfig>> frame_registration_{};
#else
protected:
  // Member variables for the frame to allow preallocation
  std::vector<types::Point<TConfig>> frame_{};
  std::vector<types::Point<TConfig>> frame_map_{};
  std::vector<types::Point<TConfig>> frame_registration_{};
#endif
};

// Explicit template instantiation
extern template class OdometryPipeline<types::ICP_EXT>;
extern template class OdometryPipeline<types::ICP_CV>;
extern template class OdometryPipeline<types::GICP_EXT>;
#ifdef __CUDACC__
extern template class OdometryPipeline<types::CUDA_ICP_EXT>;
extern template class OdometryPipeline<types::CUDA_GICP_EXT>;
#endif
}  // namespace tam::core::state
