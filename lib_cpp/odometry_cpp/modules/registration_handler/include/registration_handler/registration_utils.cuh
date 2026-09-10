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

#include <nvtx3/nvToolsExt.h>
#include <thrust/copy.h>
#include <thrust/device_vector.h>
#include <thrust/execution_policy.h>
#include <thrust/remove.h>

#include <Eigen/Dense>
#include <algorithm>
#include <vector>

#include "odometry_cuda_utils/cuda_utils.cuh"

namespace tam::core::state::cuda::utils {
/**
 * @brief Accumulate the linear system
 * @param [in] JTJ                           Jacobian transpose times Jacobian
 * @param [in] JTr                           Jacobian transpose times residual
 * @param [out] ls_flattened                 Linear system as flattened array
 */
__inline__ __device__ void accumulate_ls(
  const Eigen::Matrix<float, 6, 6>& JTJ, const Eigen::Matrix<float, 6, 1>& JTr, float* ls_flattened)
{
  int idx = 0;
#pragma unroll
  for (int r = 0; r < 6; r++) {
#pragma unroll
    for (int c = 0; c < 6; c++) {
      ls_flattened[idx++] += JTJ(r, c);
    }
  }
#pragma unroll
  for (int r = 0; r < 6; r++) {
    ls_flattened[idx++] += JTr(r);
  }
}

/**
 * @brief Kernel to reduce vector of correspondences
 * @param [in] correspondence                   Correspondences
 * @param [in] num_correspondences              Number of correspondences
 * @param [out] d_final                         Final result
 * @param [in] factor                           Factor to use for the linear system
 * @note The factor needs to be passed by value to the kernel
 * @note Passing by value is necessary to ensure that the factor is copied to each thread's local
 * memory
 */
template <typename TConfig, typename FactorType, int BLOCK_PARTIAL_SIZE>
__global__ void reduce_correspondences_kernel(
  types::Correspondence<TConfig>* correspondence, int num_correspondences, float* d_final, const FactorType factor)
{
  // local partial
  float local_sum[BLOCK_PARTIAL_SIZE] = {0.0};

  int tid = threadIdx.x;
  int stride = blockDim.x;

  // 1) Gather partial sums from all correspondences assigned to this thread
  while (tid < num_correspondences) {
    // Compute the Jacobian and residual
    factor(correspondence[tid], local_sum);
    tid += stride;
  }

  // 2) Warp-level reduction:
  // Every thread in the warp calls warp_reduce_sum on local_sum
  utils::warp_reduce_sum<BLOCK_PARTIAL_SIZE>(local_sum);

  // lane 0 of each warp holds the sum for that warp.
  int lane_id = threadIdx.x % WARP_SIZE;
  int warp_id = threadIdx.x / WARP_SIZE;  // e.g. 0..7 if blockDim.x=256

  // store warp-partials in shared memory
  __shared__ float warp_sums[BLOCK_PARTIAL_SIZE * (1024 / WARP_SIZE)];
  // Enough for up to 32 warps in a block of 1024 threads.
  // For 256 threads -> 8 warps => 8 * 42 = 336 floats.

  if (lane_id == 0) {
// Lane 0 writes out the warp sum
#pragma unroll
    for (int i = 0; i < BLOCK_PARTIAL_SIZE; i++) {
      warp_sums[warp_id * BLOCK_PARTIAL_SIZE + i] = local_sum[i];
    }
  }
  __syncthreads();

  // 3) reduce across the warp sums in shared memory (warp_id dimension).
  //    use a single warp or thread to do this final block-level sum.
  if (warp_id == 0 && lane_id == 0) {
    // Number of warps in this block
    int num_warps = blockDim.x / WARP_SIZE;  // e.g. 8 for 256 BLOCK_SIZE
    float block_sum[BLOCK_PARTIAL_SIZE] = {0.0};
#pragma unroll
    for (int w = 0; w < num_warps; w++) {
#pragma unroll
      for (int i = 0; i < BLOCK_PARTIAL_SIZE; i++) {
        block_sum[i] += warp_sums[w * BLOCK_PARTIAL_SIZE + i];
      }
    }
// 4) Write final block sum to global memory
// one block -> final block sum is the entire result
#pragma unroll
    for (int i = 0; i < BLOCK_PARTIAL_SIZE; i++) {
      d_final[i] = block_sum[i];
    }
  }
}

/**
 * @brief Build the linear system for the optimization
 * @param [in] correspondences_device       Correspondences
 * @param [in] ls_flattened_d               Linear system as flattened array
 * @param [in] factor                       Factor to use for the linear system
 * @param [in] stream                       Reference to the CUDA stream to use
 * @return                                  Linear system
 */
template <typename TConfig, typename FactorType>
__host__ types::LinearSystem build_linear_system(
  thrust::device_vector<types::Correspondence<TConfig>>& correspondences_device,
  thrust::device_vector<float>& ls_flattened_d, const FactorType& factor, ::cuda::stream_ref stream = {})
{
  nvtxRangePush("build_linear_system");
  int n = correspondences_device.size();
  static constexpr int LS_SIZE = 42;
  // Call kernel
  reduce_correspondences_kernel<TConfig, FactorType, LS_SIZE>
    <<<1, BLOCK_SIZE, 0, stream.get()>>>(thrust::raw_pointer_cast(correspondences_device.data()), n,
      thrust::raw_pointer_cast(ls_flattened_d.data()), factor);

  // Make sure everything is done
  cudaStreamSynchronize(stream.get());

  // // Copy result back to host
  std::vector<float> ls_final(LS_SIZE);
  cudaMemcpy(
    ls_final.data(), thrust::raw_pointer_cast(ls_flattened_d.data()), LS_SIZE * sizeof(float), cudaMemcpyDeviceToHost);

  // Fill linear system
  types::LinearSystem linear_system{};

  int idx = 0;
#pragma unroll
  for (int row = 0; row < 6; row++) {
#pragma unroll
    for (int col = 0; col < 6; col++) {
      linear_system.JTJ(row, col) = ls_final[idx++];
    }
  }
#pragma unroll
  for (int row = 0; row < 6; row++) {
    linear_system.JTr(row) = ls_final[idx++];
  }
  nvtxRangePop();
  return linear_system;
}

/**
 * @brief Compute the error of a set of correspondences for a given transformation
 * @param [in] correspondences_device      Correspondences
 * @param [in] pose                        Pose to compute the error for
 * @param [in] factor                      Factor to use for the error computation
 * @param [out] stream                     Reference to the CUDA stream to use
 * @return error
 */
template <typename TConfig, typename ErrorType>
__host__ float compute_error(thrust::device_vector<types::Correspondence<TConfig>>& correspondences_device,
  float* error_d, const ErrorType& factor, ::cuda::stream_ref stream = {})
{
  nvtxRangePush("compute_error");
  int n = correspondences_device.size();
  // Call kernel
  reduce_correspondences_kernel<TConfig, ErrorType, 1>
    <<<1, BLOCK_SIZE, 0, stream.get()>>>(thrust::raw_pointer_cast(correspondences_device.data()), n, error_d, factor);
  // Make sure everything is done
  cudaStreamSynchronize(stream.get());
  // Copy result back to host
  float error_h = 0.0f;
  cudaMemcpy(&error_h, error_d, sizeof(float), cudaMemcpyDeviceToHost);
  nvtxRangePop();
  return error_h;
}

/**
 * @brief Get correspondences between map and frame for given pose
 * @param [in] points                     Frame with points
 * @param [in] correspondences_device     Correspondences
 * @param [in] map                        pointer to map handler
 * @param [in] pose                       pose to get correspondences for
 * @param [in] correspondence_threshold   threshold for computation
 * @return vector of correspondences
 */
template <typename TConfig>
__host__ std::vector<types::Correspondence<TConfig>> get_correspondences(
  const thrust::device_vector<types::Point<TConfig>>& points,
  thrust::device_vector<types::Correspondence<TConfig>>& correspondences_device, const MapHandler<TConfig>* map,
  const Sophus::SE3f& pose, const float correspondence_threshold, ::cuda::stream_ref stream = {})
{
  // Transform points
  // TODO(Maxi/Marcel): Currently creating a copy here on the device, this could be avoided
  thrust::device_vector<types::Point<TConfig>> source = points;
  utils::transform_points(pose, source, stream);
  map->search_closest_neighbor(source, correspondences_device, 1, stream);

  // Check for correspondence threshold (otherwise done during build of linear system)
  auto end = thrust::remove_if(thrust::cuda::par.on(stream.get()), correspondences_device.begin(),
    correspondences_device.end(), [correspondence_threshold] __device__(const types::Correspondence<TConfig>& c) {
      return c.distance > correspondence_threshold;
    });
  correspondences_device.resize(end - correspondences_device.begin());

  // Copy correspondences to host
  std::vector<types::Correspondence<TConfig>> correspondences(correspondences_device.size());
  thrust::copy(correspondences_device.begin(), correspondences_device.end(), correspondences.begin());
  return correspondences;
}
}  // namespace tam::core::state::cuda::utils
