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
#include "odometry_cuda_utils/cuda_utils.cuh"
#endif

#include <chrono>
#include <deque>
#include <eigen3/Eigen/Core>
#include <iostream>
#include <sophus/se3.hpp>
#include <unordered_map>
#include <vector>

#include "odometry_base/odometry_base.hpp"
#include "odometry_types/odometry_types.hpp"
namespace tam::core::state
{
/**
 * @brief Filter points based on a custom predicate
 * @param [in] points             Points to filter
 * @param [in] predicate          Predicate function (returns true to REMOVE the point)
 * @param [in] stream             CUDA stream (default: default stream)
 */
#ifdef __CUDACC__
template <typename TConfig, typename Predicate>
void filter_points(
  thrust::device_vector<types::Point<TConfig>> & points, Predicate predicate,
  const std::string & name, std::unordered_map<std::string, std::int64_t> & num_points,
  std::unordered_map<std::string, double> & preprocess_time, ::cuda::stream_ref stream = {})
{
  auto start = std::chrono::high_resolution_clock::now();
  auto end =
    thrust::remove_if(thrust::cuda::par.on(stream.get()), points.begin(), points.end(), predicate);
  points.resize(end - points.begin());

  // Compute time
  // clang-format off
  double time = std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::high_resolution_clock::now() - start).count() * 1.0e-3;  // NOLINT
  // clang-format on
  // Update debug information
  num_points[name] = static_cast<std::int64_t>(points.size());
  preprocess_time[name] = time;
}
#else
template <typename TConfig, typename Predicate>
void filter_points(
  std::vector<types::Point<TConfig>> & points, Predicate predicate, const std::string & name,
  std::unordered_map<std::string, std::int64_t> & num_points,
  std::unordered_map<std::string, double> & preprocess_time)
{
  auto start = std::chrono::high_resolution_clock::now();
  points.erase(std::remove_if(points.begin(), points.end(), predicate), points.end());
  // Compute time
  double time = std::chrono::duration_cast<std::chrono::microseconds>(
                  std::chrono::high_resolution_clock::now() - start)
                  .count() *
                1.0e-3;  // NOLINT
  // Update debug information
  num_points[name] = static_cast<std::int64_t>(points.size());
  preprocess_time[name] = time;
}
#endif
/**
 * @brief Base class for preprocessing handlers
 */
template <typename TConfig>
class PreprocessingHandler
: public OdometryBase<TConfig, types::PreprocessingConfig, types::PreprocessingDebug>
{
public:
  /**
   * @brief Preprocess a frame of points
   * @param [in,out] frame          Frame to preprocess
   * @param [in] voxel_size         Size of the voxel for downsampling
   */
#ifdef __CUDACC__
  virtual __host__ bool preprocess(
    thrust::device_vector<types::Point<TConfig>> & frame, ::cuda::stream_ref stream = {}) = 0;
#else
  virtual bool preprocess(std::vector<types::Point<TConfig>> & frame) = 0;
#endif
  /**
   * @brief Initialize parameters
   */
  virtual void init() = 0;

protected:
  // Inherit constructor from OdometryBase for param manager and logger
  PreprocessingHandler(tam::pmg::ParamReferenceManager * pmg, tam::tsl::ReferenceLogger * logger)
  : OdometryBase<TConfig, types::PreprocessingConfig, types::PreprocessingDebug>(pmg, logger)
  {
    this->set_config(pmg);
    this->set_logging(logger);
    // Additional initialization
  }
  // Inherit constructor from OdometryBase for config and debug object
  PreprocessingHandler(
    const types::PreprocessingConfig & config, const types::PreprocessingDebug & debug)
  : OdometryBase<TConfig, types::PreprocessingConfig, types::PreprocessingDebug>(config, debug)
  {
  }
  /**
   * @brief Set the configuration of the preprocessing handler from the param manager
   * @param [in] pmg                Param manager
   */
  void set_config(tam::pmg::ParamReferenceManager * pmg) override
  {
    // clang-format off
    pmg->declare_parameter("preprocessing.crop_range", &this->config_.crop_range, std::vector<double>{0.0, 100.0}, tam::pmg::ParameterType::DOUBLE_ARRAY, "Range crop bounds [min, max] (m)");  // NOLINT
      // Vehicle footprint filter
    pmg->declare_parameter("preprocessing.crop_footprint", &this->config_.crop_footprint, std::vector<double>{6.0, 1.0}, tam::pmg::ParameterType::DOUBLE_ARRAY, "Vehicle footprint half-extents [lon, lat] (m)");  // NOLINT
    if constexpr (types::HASRADAR<TConfig>) {
      // Only declare radar preprocessing parameters if the point type has radar attributes
      // Spherical cropping
      pmg->declare_parameter("preprocessing.radar.crop_spherical", &this->config_.radar.crop_spherical, std::vector<double>{-std::numbers::pi, std::numbers::pi, -std::numbers::pi / 2, std::numbers::pi / 2}, tam::pmg::ParameterType::DOUBLE_ARRAY, "Spherical crop bounds [az_min, az_max, el_min, el_max] (rad)");  // NOLINT
      // Min/max filtering for RCS
      pmg->declare_parameter("preprocessing.radar.rcs_threshold", &this->config_.radar.rcs_threshold, std::vector<double>{-std::numeric_limits<double>::infinity(), std::numeric_limits<double>::infinity()}, tam::pmg::ParameterType::DOUBLE_ARRAY, "Enable min/max RCS filtering");  // NOLINT
      // Min/max filtering for SNR
      pmg->declare_parameter("preprocessing.radar.snr_threshold", &this->config_.radar.snr_threshold, std::vector<double>{-std::numeric_limits<double>::infinity(), std::numeric_limits<double>::infinity()}, tam::pmg::ParameterType::DOUBLE_ARRAY, "Enable min/max SNR filtering");  // NOLINT
      // Min/max filtering for velocity
      pmg->declare_parameter("preprocessing.radar.velocity_threshold", &this->config_.radar.velocity_threshold, std::vector<double>{-std::numeric_limits<double>::infinity(), std::numeric_limits<double>::infinity()}, tam::pmg::ParameterType::DOUBLE_ARRAY, "Enable min/max velocity filtering");  // NOLINT
      // Min/max filtering for confidence
      pmg->declare_parameter("preprocessing.radar.confidence_threshold", &this->config_.radar.confidence_threshold, std::vector<double>{-std::numeric_limits<double>::infinity(), std::numeric_limits<double>::infinity()}, tam::pmg::ParameterType::DOUBLE_ARRAY, "Enable min/max confidence filtering");  // NOLINT
      pmg->declare_parameter("preprocessing.radar.jitter_method", &this->config_.radar.jitter_method, std::string{"AdaptiveNearestNeighbour"}, tam::pmg::ParameterType::STRING, "Jitter filter method");  // NOLINT
      // AdaptiveNearestNeighbour parameters
      pmg->declare_parameter("preprocessing.radar.nn_radius", &this->config_.radar.nn_radius, 3.0, tam::pmg::ParameterType::DOUBLE, "Constant radius for all distances (m)");  // NOLINT
      pmg->declare_parameter("preprocessing.radar.relaxation_coefficient_t", &this->config_.radar.relaxation_coefficient_t, 1.0, tam::pmg::ParameterType::DOUBLE, "Relaxation coefficient t");  // NOLINT
      pmg->declare_parameter("preprocessing.radar.max_min_neighbors", &this->config_.radar.max_min_neighbors, static_cast<std::int64_t>(20), tam::pmg::ParameterType::INTEGER, "Max value for min_neighbors");  // NOLINT
      pmg->declare_parameter("preprocessing.radar.min_min_neighbors", &this->config_.radar.min_min_neighbors, static_cast<std::int64_t>(1), tam::pmg::ParameterType::INTEGER, "Min value for min_neighbors (lower bound)");  // NOLINT
    }
    // clang-format on
  }
  /**
   * @brief Register the debug variables with the logger
   * @param [in] logger             Logger
   */
  void set_logging(tam::tsl::ReferenceLogger * logger) const override
  {
    logger->log("preprocessing/num_points_preprocessed", &this->debug_.num_points_preprocessed);
    logger->log("preprocessing/preprocess_time", &this->debug_.preprocess_time);
  }

public:
#ifdef __CUDACC__
  /**
   * @brief Crop points given a minimum and maximum range
   * @param [in] points             Points to crop
   * @param [in] stream             CUDA stream (default: default stream)
   */
  void crop_points(
    thrust::device_vector<types::Point<TConfig>> & points, ::cuda::stream_ref stream = {})
  {
    double min_range = this->config_.crop_range.front();
    double max_range = this->config_.crop_range.back();

    filter_points(
      points,
      [min_range, max_range] __device__(const types::Point<TConfig> & point) {
        float range = point.pos.norm();
        return range < min_range || range > max_range;
      },
      "crop_range", this->debug_.num_points_preprocessed, this->debug_.preprocess_time, stream);
  }
  /**
   * @brief Crop points within the vehicle footprint
   * @param [in] frame              Points to crop
   * @param [in] stream             CUDA stream (default: default stream)
   */
  void crop_footprint(
    thrust::device_vector<types::Point<TConfig>> & frame, ::cuda::stream_ref stream = {})
  {
    double footprint_lon = this->config_.crop_footprint.front();
    double footprint_lat = this->config_.crop_footprint.back();
    filter_points(
      frame,
      [footprint_lat, footprint_lon] __device__(const types::Point<TConfig> & p) {
        // Check if point is within vehicle footprint (rectangular box)
        bool within_footprint =
          (std::abs(p.pos.x()) <= footprint_lon &&  // rear and forward boundary
           std::abs(p.pos.y()) <= footprint_lat);   // left and right boundary
        return within_footprint;
      },
      "crop_footprint", this->debug_.num_points_preprocessed, this->debug_.preprocess_time, stream);
  }
#else
  void crop_points(std::vector<types::Point<TConfig>> & points)
  {
    filter_points(
      points,
      [this](const auto & point) {
        double range = point.pos.norm();
        return range < this->config_.crop_range.front() || range > this->config_.crop_range.back();
      },
      "crop_range", this->debug_.num_points_preprocessed, this->debug_.preprocess_time);
  }
  void crop_footprint(std::vector<types::Point<TConfig>> & frame)
  {
    filter_points(
      frame,
      [this](const auto & p) {
        // Check if point is within vehicle footprint (rectangular box)
        bool within_footprint =
          (std::abs(p.pos.x()) <=
             this->config_.crop_footprint.front() &&  // rear and forward boundary
           std::abs(p.pos.y()) <= this->config_.crop_footprint.back());  // left and right boundary
        return within_footprint;
      },
      "crop_footprint", this->debug_.num_points_preprocessed, this->debug_.preprocess_time);
  }
#endif
};
}  // namespace tam::core::state
