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
#include <thrust/device_vector.h>
#endif

#include <Eigen/Dense>
#include <chrono>
#include <vector>

#include "odometry_types/point_types.hpp"
namespace tam::core::state
{
/**
 * @brief Base functor for building the linear system for the registration algorithm
 * @param [in] correspondence         Correspondence
 * @return                             Linear system
 * @note                               This is a functor to be used with the parallel
 *                                     reduction in the build_linear_system function
 */
template <typename TConfig>
struct FactorBase
{
public:
  /**
   * @brief Function to compute linear system for a given correspondence
   * @param [in] correspondence Correspondence to compute linear system for
   * @param [out] ls_flattened  Accumulated linear system as flattened array (CUDA only)
   * @return                    Linear system (CPU only)
   */
#ifdef __CUDACC__
  __device__ virtual void operator()(
    types::Correspondence<TConfig> & correspondence, float * ls_flattened) const = 0;
#else
  virtual types::LinearSystem operator()(types::Correspondence<TConfig> & correspondence) const = 0;
#endif
  /**
   * @brief Set the kernel scale
   * @param [in] kernel_scale Scale of the robust kernel
   */
  void set_kernel_scale(const float kernel_scale) { kernel_scale_ = kernel_scale; }
  /**
   * @brief Get the kernel scale
   * @return Scale of the robust kernel
   */
  float get_kernel_scale() const { return kernel_scale_; }
  /**
   * @brief Set correspondence threshold
   * @param [in] correspondence_threshold Threshold for correspondences
   */
  void set_correspondence_threshold(const float correspondence_threshold)
  {
    correspondence_threshold_ = correspondence_threshold;
  }
  /**
   * @brief Get the correspondence threshold
   * @return Correspondence threshold
   */
  float get_correspondence_threshold() const { return correspondence_threshold_; }

protected:
  float kernel_scale_{0.0f};
  float correspondence_threshold_{0.0f};
};
/**
 * @brief Base functor for computing the error of a correspondence for a given transformation
 * @param [in] correspondence Correspondence
 * @param [in] T              Transformation to apply
 * @return                    Error of the correspondence
 */
template <typename TConfig>
struct ErrorBase
{
public:
  /**
   * @brief Function to compute error for a given correspondence
   * @param [in] correspondence Correspondence to compute error for
   * @param [out] sum           Accumulated error (CUDA only)
   * @return                    Error of the correspondence (CPU only)
   */
#ifdef __CUDACC__
  __device__ virtual void operator()(
    const types::Correspondence<TConfig> & correspondence, float * sum) const = 0;
#else
  virtual float operator()(const types::Correspondence<TConfig> & correspondence) const = 0;
#endif
  /**
   * @brief Set transformation
   * @param [in] T Transformation to apply
   */
  void set_transform(const Sophus::SE3f & T) { T_ = T; }
  /**
   * @brief Get the transformation
   * @return Transformation to apply
   */
  Sophus::SE3f get_transform() const { return T_; }
  /**
   * @brief Set correspondence threshold
   * @param [in] correspondence_threshold Threshold for correspondences
   */
  void set_correspondence_threshold(const float correspondence_threshold)
  {
    correspondence_threshold_ = correspondence_threshold;
  }
  /**
   * @brief Get the correspondence threshold
   * @return Correspondence threshold
   */
  float get_correspondence_threshold() const { return correspondence_threshold_; }

protected:
  Sophus::SE3f T_{};
  float correspondence_threshold_{0.0f};
};
};  // namespace tam::core::state
