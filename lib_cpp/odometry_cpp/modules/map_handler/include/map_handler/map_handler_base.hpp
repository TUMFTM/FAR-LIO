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

#include "map_handler/map_utils.cuh"
#include "odometry_cuda_utils/cuda_utils.cuh"
#endif

#include <tbb/blocked_range.h>
#include <tbb/concurrent_vector.h>
#include <tbb/global_control.h>
#include <tbb/info.h>
#include <tbb/parallel_for.h>
#include <tbb/task_arena.h>

#include <algorithm>
#include <atomic>
#include <chrono>
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Dense>
#include <map>
#include <memory>
#include <mutex>
#include <numbers>
#include <sophus/se3.hpp>
#include <tuple>
#include <variant>
#include <vector>

#include "odometry_base/odometry_base.hpp"
#include "odometry_types/point_types.hpp"
#include "odometry_utils/utils.hpp"
namespace tam::core::state
{
/**
 * @brief Base class for map handling
 */
template <typename TConfig>
class MapHandler : public OdometryBase<TConfig, types::MapConfig, types::MapDebug>
{
public:
  /**
   * @brief Clear the map
   */
#ifdef __CUDACC__
  virtual __host__ void clear() = 0;
#else
  virtual void clear() = 0;
#endif
  /**
   * @brief                         Check if map is empty
   * @return                        True if map is empty
   */
#ifdef __CUDACC__
  virtual __host__ bool empty() const = 0;
#else
  virtual bool empty() const = 0;
#endif
  /**
   * @brief Get the amount of points in the map
   * @return                        Number of points in the map
   */
#ifdef __CUDACC__
  virtual __host__ std::size_t num_points(::cuda::stream_ref stream = {}) const = 0;
#else
  virtual std::size_t num_points() const = 0;
#endif
/**
 * @brief search for closest neighbors of a vector of points
 * @param [in] points                      Points to search for
 * @param [in] correspondences             Vector to return the correspondences in
 * @param [in] adjacent_voxels             Number of adjacent voxels to search in
 * @param [in] (stream)                    Reference to the CUDA stream to use
 */
#ifdef __CUDACC__
  virtual __host__ void search_closest_neighbor(
    thrust::device_vector<types::Point<TConfig>> & points,
    thrust::device_vector<types::Correspondence<TConfig>> & correspondences,
    const int16_t adjacent_voxels = 1, ::cuda::stream_ref stream = {}) const = 0;
#else
  virtual types::Correspondence<TConfig> search_closest_neighbor(
    const types::Point<TConfig> & point, const int16_t adjacent_voxels = 1) const = 0;
#endif
/**
 * @brief Add points to the map
 * @param [in] points             Points to add
 * @param [in] map_density        Map density to use for adding points (either max points per voxel
 * or adaptive map density)
 * @param [in] adjacent_voxels    Number of adjacent voxels to search for
 * @param [in] num_neighbors      Number of neighbors to search for
 * @param [in] use_active_map     Flag to indicate if the active or inactive map should be used
 * @param [in] (stream)           Reference to the CUDA stream to use
 */
#ifdef __CUDACC__
  virtual __host__ void add_points(
    const std::vector<types::Point<tam::core::state::types::Point_XYZ>> & points,
    const std::variant<int16_t, types::AdaptiveMapDensity> & map_density =
      TConfig::MAX_POINTS_PER_VOXEL,
    const int16_t adjacent_voxels = 1, const int16_t num_neighbors = TConfig::NUM_NEIGHBORS,
    const bool use_active_map = true, ::cuda::stream_ref stream = {}) = 0;
  virtual __host__ void add_points_device(
    const thrust::device_vector<types::Point<tam::core::state::types::Point_XYZ>> & points,
    const std::variant<int16_t, types::AdaptiveMapDensity> & map_density =
      TConfig::MAX_POINTS_PER_VOXEL,
    const int16_t adjacent_voxels = 1, const int16_t num_neighbors = TConfig::NUM_NEIGHBORS,
    const bool use_active_map = true, ::cuda::stream_ref stream = {}) = 0;
#else
  virtual void add_points(
    const std::vector<types::Point<tam::core::state::types::Point_XYZ>> & points,
    const std::variant<int16_t, types::AdaptiveMapDensity> & map_density =
      TConfig::MAX_POINTS_PER_VOXEL,
    const int16_t adjacent_voxels = 1, const int16_t num_neighbors = TConfig::NUM_NEIGHBORS,
    const bool use_active_map = true) = 0;
#endif
/**
 * @brief Update the map with new points
 * @param [in] points             Points to update
 * @param [in] pose               Pose the points are in
 * @param [in] map_density        Map density to use for adding points (either max points per voxel
 * or adaptive map density)
 * @param [in] adjacent_voxels    Number of adjacent voxels to search for
 * @param [in] num_neighbors      Number of neighbors to search for
 * @param [in] (stream)           Reference to the CUDA stream to use
 */
#ifdef __CUDACC__
  virtual __host__ void update_points(
    const thrust::device_vector<types::Point<TConfig>> & points, const Sophus::SE3f & pose,
    const std::variant<int16_t, types::AdaptiveMapDensity> & map_density =
      TConfig::MAX_POINTS_PER_VOXEL,
    const int16_t adjacent_voxels = 1, const int16_t num_neighbors = TConfig::NUM_NEIGHBORS,
    ::cuda::stream_ref stream = {}) = 0;
#else
  virtual void update_points(
    const std::vector<types::Point<TConfig>> & points, const Sophus::SE3f & pose,
    const std::variant<int16_t, types::AdaptiveMapDensity> & map_density =
      TConfig::MAX_POINTS_PER_VOXEL,
    const int16_t adjacent_voxels = 1, const int16_t num_neighbors = TConfig::NUM_NEIGHBORS) = 0;
#endif

  /**
   * @brief Get the map density
   * @param [in] range                Range threshold for max density
   * @return                          Adaptive Map density
   */
#ifdef __CUDACC__
  virtual __host__ types::AdaptiveMapDensity get_density(
    double range, ::cuda::stream_ref stream = {}) = 0;
#else
  virtual types::AdaptiveMapDensity get_density(double range) = 0;
#endif
  /**
   * @brief Get the map as a point cloud
   * @return                        Point cloud of the map
   */
#ifdef __CUDACC__
  virtual __host__ thrust::device_vector<types::Point<TConfig>> get_cloud(
    ::cuda::stream_ref stream = {}) const = 0;
#else
  virtual std::vector<types::Point<TConfig>> get_cloud() const = 0;
#endif
  /**
   * @brief Get the vector holding the neighbors of the points in the map
   * @return                        Neighbors of the points in the map
   * @note this is required for testing of the muliple nearest neighbor search
   */
#ifdef __CUDACC__
  virtual __host__ thrust::device_vector<types::Neighbors<TConfig>> get_neighbors() const = 0;
#else
  virtual std::vector<types::Neighbors<TConfig>> get_neighbors() const = 0;
#endif
  /**
   * @brief Switch the active map
   */
  virtual void switch_active_map() = 0;
  /**
   * @brief Mutex that serializes async-update writes to the inactive map
   *        against any in-place use of the inactive slot (e.g. swap_map on
   *        the CUDA backend).
   */
  std::mutex & get_mutex() { return this->mutex_; }
  /**
   * @brief Function to request switch between active maps.
   *        Requires filling the inactive map slot beforehand.
   */
  void request_map_switch() { this->switch_pending_.store(true); }
  /**
   * @brief Switch active map under mutex (e.g. if updated asynchronously).
   */
  void consume_map_switch()
  {
    if (!switch_pending_.load()) return;
    std::lock_guard<std::mutex> lock(this->mutex_);
    if (!switch_pending_.load()) return;
    this->switch_active_map();
    switch_pending_.store(false);
  }
  /**
   * @brief Get the map resolution
   * @return                        Map resolution
   */
  double get_resolution() const
  {
    return std::sqrt(
      this->config_.voxel_size * this->config_.voxel_size / TConfig::MAX_POINTS_PER_VOXEL);
  }
  /**
   * @brief Initialize threading
   * -> needs to be done after param override
   */
  void init(const std::size_t num_threads = 1, [[maybe_unused]] const bool update_map = false)
  {
    // This global variable requires static duration storage to be able to manipulate the max
    // concurrency from TBB across the entire class
    static const auto tbb_control_settings = tbb::global_control(
      tbb::global_control::max_allowed_parallelism, static_cast<size_t>(num_threads));
    // Set cov regularization type
    if (this->config_.cov_regularization == "SVD")
      this->cov_regularization_type_ = types::CovRegularizationType::SVD;
    else if (this->config_.cov_regularization == "FROBENIUS")
      this->cov_regularization_type_ = types::CovRegularizationType::FROBENIUS;
    else if (this->config_.cov_regularization == "MIN_EIGENVALUE")
      this->cov_regularization_type_ = types::CovRegularizationType::MIN_EIGENVALUE;
    else
      std::runtime_error(
        "Invalid covariance regularization type: " + this->config_.cov_regularization);
#ifdef __CUDACC__
    // Allocate memory for CUDA vectors
    this->allocate_memory(update_map, 50000);
#endif
  }

protected:
  // Inherit constructor from OdometryBase for param manager and logger
  MapHandler(tam::pmg::ParamReferenceManager * pmg, tam::tsl::ReferenceLogger * logger)
  : OdometryBase<TConfig, types::MapConfig, types::MapDebug>(pmg, logger)
  {
    this->set_config(pmg);
    this->set_logging(logger);
    // Additional initialization
  }
  // Inherit constructor from OdometryBase for config and debug object
  MapHandler(const types::MapConfig & config, const types::MapDebug & debug)
  : OdometryBase<TConfig, types::MapConfig, types::MapDebug>(config, debug)
  {
    // Additional initialization
  }
  /**
   * @brief Set the configuration of the map from the param manager
   * @param [in] pmg                Param manager
   */
  void set_config(tam::pmg::ParamReferenceManager * pmg) override
  {
    // clang-format off
    pmg->declare_parameter("map.frame_map", &this->config_.frame_map, false, tam::pmg::ParameterType::BOOL, "Keep only the last added frame instead of building a local map");  // NOLINT
    pmg->declare_parameter("map.voxel_size", &this->config_.voxel_size, 1.0, tam::pmg::ParameterType::DOUBLE, "Voxel edge length (m)");  // NOLINT
    pmg->declare_parameter("map.max_distance", &this->config_.max_distance, 50.0, tam::pmg::ParameterType::DOUBLE, "Max distance from current pose to keep voxels in the local map (m)");  // NOLINT
    pmg->declare_parameter("map.cov_regularization", &this->config_.cov_regularization, "SVD", tam::pmg::ParameterType::STRING, "Covariance regularization: SVD, FROBENIUS or MIN_EIGENVALUE");  // NOLINT
    // clang-format on
  }
  /**
   * @brief Register the debug variables with the logger
   * @param [in] logger             Logger
   */
  void set_logging(tam::tsl::ReferenceLogger * logger) const override
  {
    logger->log("map/num_voxel", &this->debug_.num_voxel);
    logger->log("map/num_points", &this->debug_.num_points);
    logger->log("map/points_added", &this->debug_.points_added);
    logger->log("map/points_removed", &this->debug_.points_removed);
    logger->log("map/voxel_added", &this->debug_.voxel_added);
    logger->log("map/voxel_removed", &this->debug_.voxel_removed);
    logger->log("map/update_time", &this->debug_.update_time);
    logger->log("map/normal_cov_time", &this->debug_.normal_cov_time);
    logger->log("map/load_factor", &this->debug_.load_factor);
    logger->log("map/conditional", &this->debug_.conditional);
  }
  /**
   * @brief Allocate memory for map instance
   * @param [in] update_map          Flag to indicate if the map is being updated
   * @param [in] size                Size to allocate
   */
  virtual void allocate_memory(
    [[maybe_unused]] const bool update_map, [[maybe_unused]] const size_t size) = 0;

protected:
/**
 * @brief search for closest neighbors of a point
 * @brief Amount of neighbors to search for is defined by TConfig::NUM_NEIGHBORS
 * @param [in] (points)                   Point to search for
 * @param [in] correspondences          Vector to return the correspondences in
 * @param [in] adjacent_voxels          Number of adjacent voxels to search in
 * @param [in] num_neighbors            Number of neighbors to search for
 * @param [in] use_active_map           Flag to indicate if the active or inactive map should be
 * @note Callers using use_active_map = false are expected to hold the base class's
 *       map_async_update_mutex during the call to ensure that no concurrent operations intervene.
 *       used
 * @param [in] (stream)                 Reference to the CUDA stream to use
 */
#ifdef __CUDACC__
  virtual __host__ void search_closest_neighbors(
    thrust::device_vector<types::Neighbors<TConfig>> & neighbors, const int16_t adjacent_voxels,
    const int16_t num_neighbors, const bool use_active_map = true,
    ::cuda::stream_ref stream = {}) = 0;
#else
  virtual void search_closest_neighbors(
    types::Neighbors<TConfig> & point, const int16_t adjacent_voxels, const int16_t num_neighbors,
    const bool use_active_map = true) const = 0;
#endif
/**
 * @brief Compute normals and covariances for a point cloud
 * @brief Adapted from:
 * https://github.com/koide3/small_gicp/blob/master/include/small_gicp/util/normal_estimation.hpp
 * @param [in] point               Point to compute normals and covariance for
 */
#ifdef __CUDACC__
  /**
   * @brief Set the normal and covariance of a point given its neighbors
   * @param [in] correspondences    Correspondences of the point cloud
   * @param [in] num_neighbors      Number of neighbors to consider for normals/covariances
   * @param [in] num_multiprocessors Amount of multiprocessors of the GPU
   * @param [in] stream             Reference to the CUDA stream to use
   */
  __host__ void set_normal_covariance(
    thrust::device_vector<types::Neighbors<TConfig>> & neighbors, const int16_t num_neighbors,
    const size_t num_multiprocessors, ::cuda::stream_ref stream = {})
    requires types::HASNORMALCOV<TConfig>
  {
    nvtxRangePush("set_normal_covariance");
    auto grid_size = std::min(
      4 * num_multiprocessors, (neighbors.size() + cuda::BLOCK_SIZE - 1) / cuda::BLOCK_SIZE);

    types::Neighbors<TConfig> * raw_neighbors = thrust::raw_pointer_cast(neighbors.data());

    cuda::utils::set_normal_covariance_kernel<<<grid_size, cuda::BLOCK_SIZE, 0, stream.get()>>>(
      raw_neighbors, neighbors.size(), this->cov_regularization_type_, num_neighbors);

    stream.wait();
    nvtxRangePop();
  }
#else
  void set_normal_covariance(types::Neighbors<TConfig> & neighbor, const int16_t num_neighbors)
    requires types::HASNORMALCOV<TConfig>
  {
    // Check if enough neighbors were found and set the normal to zero if not
    if (neighbor.num_neighbors < num_neighbors) {
      neighbor.point->normal = Eigen::Vector3f::Zero();
      neighbor.point->cov = Eigen::Matrix3f::Zero();
      return;
    }
    Eigen::Vector3f sum_pts = Eigen::Vector3f::Zero();
    Eigen::Matrix3f sum_pts_cov = Eigen::Matrix3f::Zero();

    // clang-format off
    for (size_t i = 0; i < neighbor.num_neighbors; ++i) sum_pts += neighbor.neighbor[i]->pos;
    const Eigen::Vector3f mean = sum_pts / static_cast<float>(neighbor.num_neighbors);
    for (size_t i = 0; i < neighbor.num_neighbors; ++i)
      sum_pts_cov += (neighbor.neighbor[i]->pos - mean) * (neighbor.neighbor[i]->pos - mean).transpose();  // NOLINT
    const Eigen::Matrix3f cov = sum_pts_cov / static_cast<float>(neighbor.num_neighbors);
    // clang-format on
    // Compute the normal
    Eigen::SelfAdjointEigenSolver<Eigen::Matrix3f> eigensolver;
    eigensolver.computeDirect(cov);

    // Set normal and covariance
    const Eigen::Vector3f normal = eigensolver.eigenvectors().col(0).normalized();
    if (neighbor.point->pos.dot(normal) > 0) {
      neighbor.point->normal = -normal;
    } else {
      neighbor.point->normal = normal;
    }
    /**
     * Covariance regularizations from
     * https://github.com/koide3/small_gicp/blob/master/include/small_gicp/util/normal_estimation.hpp
     * https://github.com/koide3/fast_gicp/blame/master/src/fast_gicp/cuda/covariance_regularization.cu
     */
    switch (this->cov_regularization_type_) {
      case types::CovRegularizationType::SVD: {
        // SVD regularization
        const Eigen::Vector3f values(1.0e-3, 1.0, 1.0);
        neighbor.point->cov =
          eigensolver.eigenvectors() * values.asDiagonal() * eigensolver.eigenvectors().transpose();
        break;
      }
      case types::CovRegularizationType::FROBENIUS: {
        // Frobenius regularization
        float lambda = 1e-3;
        Eigen::Matrix3f C = cov + lambda * Eigen::Matrix3f::Identity();
        Eigen::Matrix3f C_inv = C.inverse();
        Eigen::Matrix3f C_norm = (C_inv / C_inv.norm()).inverse();
        neighbor.point->cov = C_norm;
        break;
      }
      case types::CovRegularizationType::MIN_EIGENVALUE: {
        // Min eigenvalue regularization
        const Eigen::Vector3f eigen_values = eigensolver.eigenvalues();
        const Eigen::Matrix3f eigen_vectors = eigensolver.eigenvectors();

        // Create regularized eigenvalues
        Eigen::Vector3f regularized_values;
        regularized_values[0] = fmaxf(1.0e-3, eigen_values[0]);
        regularized_values[1] = fmaxf(1.0e-3, eigen_values[1]);
        regularized_values[2] = fmaxf(1.0e-3, eigen_values[2]);

        // Reconstruct regularized covariance directly
        neighbor.point->cov =
          eigen_vectors * regularized_values.asDiagonal() * eigen_vectors.transpose();
        break;
      }
      default: {
        // Default case: use the identity matrix
        neighbor.point->cov = Eigen::Matrix3f::Identity();
      }
    }
    return;
  }
#endif
protected:
  // Mutex to handle async map updates via the inactive map
  // slot while the active map is being used for registration.
  std::mutex mutex_;
  // Atomic variable to track which map is active
  std::atomic<bool> map_a_active_{true};
  // Signals that the async worker has staged a new map in the inactive slot
  // and waits for it to be consumed
  std::atomic<bool> switch_pending_{false};
  types::CovRegularizationType cov_regularization_type_{
    types::CovRegularizationType::SVD};  // Default regularization type
};
}  // namespace tam::core::state
