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

#include <cuda_runtime.h>
#include <thrust/device_vector.h>

#include <algorithm>
#include <cuda/stream_ref>
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Dense>
#include <sophus/se3.hpp>
#include <string>

#include "odometry_types/map.hpp"
#include "odometry_types/point_types.hpp"

namespace tam::core::state::cuda::utils {
/**
 * @brief Kernel to set the normal and covariance of a point given its neighbors
 * @param [in] neighbors          Raw pointer to the neighbors vector
 * @param [in] num_points         Number of points in the input point cloud
 * @param [in] cov_regularization Covariance regularization type
 * @param [in] num_neighbors      Number of neighbors per point
 */
template <typename TConfig>
__global__ void set_normal_covariance_kernel(types::Neighbors<TConfig>* neighbors, const size_t num_points,
  const types::CovRegularizationType cov_regularization, const int16_t num_neighbors)
{
  auto tid = threadIdx.x + blockIdx.x * blockDim.x;

  while (tid < num_points) {
    if (neighbors[tid].num_neighbors < num_neighbors) {
      neighbors[tid].point->normal = Eigen::Vector3f::Zero();
      neighbors[tid].point->cov = Eigen::Matrix3f::Zero();
    } else {
      Eigen::Vector3f sum_pts = Eigen::Vector3f::Zero();
      Eigen::Matrix3f sum_pts_cov = Eigen::Matrix3f::Zero();
      // clang-format off
      for (int16_t i = 0; i < neighbors[tid].num_neighbors; ++i) sum_pts += neighbors[tid].neighbor[i]->pos;  // NOLINT
      const Eigen::Vector3f mean = sum_pts / static_cast<float>(neighbors[tid].num_neighbors);
      for (int16_t i = 0; i < neighbors[tid].num_neighbors; ++i)
        sum_pts_cov += (neighbors[tid].neighbor[i]->pos - mean) * (neighbors[tid].neighbor[i]->pos - mean).transpose();  // NOLINT
      const Eigen::Matrix3f cov = sum_pts_cov / static_cast<float>(neighbors[tid].num_neighbors);

      // Compute the normal
      Eigen::SelfAdjointEigenSolver<Eigen::Matrix3f> eigensolver;
      eigensolver.computeDirect(cov);

      // Set normal and covariance
      const Eigen::Vector3f normal = eigensolver.eigenvectors().col(0).normalized();
      if (neighbors[tid].point->pos.dot(normal) > 0) {
        neighbors[tid].point->normal = -normal;
      } else {
        neighbors[tid].point->normal = normal;
      }
      /**
       * Covariance Regularization based on
       * https://github.com/koide3/fast_gicp/blame/master/src/fast_gicp/cuda/covariance_regularization.cu
       * https://github.com/koide3/small_gicp/blob/master/include/small_gicp/util/normal_estimation.hpp
       */
      switch (cov_regularization) {
        case types::CovRegularizationType::SVD: {
          // SVD regularization (Default as highlighted in Segal et. al)
          // Note! Calling vector.asDiagonal() here does not work on the device
          Eigen::Matrix3f values = Eigen::Matrix3f::Zero();
          values(0, 0) = 1.0e-3;
          values(1, 1) = 1.0;
          values(2, 2) = 1.0;
          Eigen::Matrix3f v_inv = eigensolver.eigenvectors().transpose();
          neighbors[tid].point->cov = eigensolver.eigenvectors() * values * v_inv;
          break;
        }
        case types::CovRegularizationType::FROBENIUS: {
          // Frobenius regularization
          float lambda = 1e-3;
          Eigen::Matrix3f C = cov + lambda * Eigen::Matrix3f::Identity();
          Eigen::Matrix3f C_inv = C.inverse();
          Eigen::Matrix3f C_norm = (C_inv / C_inv.norm()).inverse();
          neighbors[tid].point->cov = C_norm;
          break;
        }
        case types::CovRegularizationType::MIN_EIGENVALUE: {
          // Min eigenvalue regularization
          const Eigen::Vector3f eigen_values = eigensolver.eigenvalues();
          const Eigen::Matrix3f eigen_vectors = eigensolver.eigenvectors();

          // Create regularized eigenvalues
          Eigen::Matrix3f regularized_values = Eigen::Matrix3f::Zero();
          regularized_values(0, 0) = fmaxf(1.0e-3, eigen_values[0]);
          regularized_values(1, 1) = fmaxf(1.0e-3, eigen_values[1]);
          regularized_values(2, 2) = fmaxf(1.0e-3, eigen_values[2]);

          // Reconstruct regularized covariance directly
          neighbors[tid].point->cov =
            eigen_vectors * regularized_values * eigen_vectors.transpose();
          break;
        }
        default: {
          // Default case: use the identity matrix
          neighbors[tid].point->cov = Eigen::Matrix3f::Identity();
        }
      }
    }
    tid += gridDim.x * blockDim.x;
  }
}
}  // namespace tam::core::state::cuda::utils
