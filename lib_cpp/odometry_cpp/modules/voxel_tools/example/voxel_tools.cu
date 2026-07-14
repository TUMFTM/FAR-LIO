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

#include <chrono>
#include <iostream>
#include <random>
#include <vector>

#include "odometry_types/odometry_config.hpp"
#include "odometry_types/point_types.hpp"
#include "voxel_tools/voxel_tools.cuh"
#include "voxel_tools/voxel_tools.hpp"
void generate_random_points(
  std::vector<tam::core::state::types::Point<tam::core::state::types::CUDA_ICP_EXT>> & frame,
  size_t num_points)
{
  std::random_device rd;
  std::mt19937 gen(rd());
  std::uniform_real_distribution<> dis(-100.0, 100.0);

  for (size_t i = 0; i < num_points; ++i) {
    tam::core::state::types::Point<tam::core::state::types::CUDA_ICP_EXT> point;
    point.pos = Eigen::Vector3f(dis(gen), dis(gen), dis(gen));
    frame.push_back(point);
  }
}
int main()
{
  // Define a frame of points
  std::vector<tam::core::state::types::Point<tam::core::state::types::CUDA_ICP_EXT>> frame;
  generate_random_points(frame, 100000);
  cudaDeviceSynchronize();

  // Downsample the frame to a voxel grid and print the downsampled frame
  auto start_time = std::chrono::high_resolution_clock::now();
  std::vector<tam::core::state::types::Point<tam::core::state::types::CUDA_ICP_EXT>>
    frame_downsampled_cuda =
      tam::core::state::cuda::voxel_downsample<tam::core::state::types::CUDA_ICP_EXT>(frame, 1.0);
  auto end_time = std::chrono::high_resolution_clock::now();

  // Calculate the duration
  std::chrono::duration<double, std::milli> duration = end_time - start_time;
  std::cout << "Time taken for voxel_downsample cuda: " << duration.count() << " ms." << std::endl;

  for (int i = 0; i < 10; i++) {
    start_time = std::chrono::high_resolution_clock::now();
    std::vector<tam::core::state::types::Point<tam::core::state::types::CUDA_ICP_EXT>>
      frame_downsampled_cuda =
        tam::core::state::cuda::voxel_downsample<tam::core::state::types::CUDA_ICP_EXT>(
          frame, 1.0);
    end_time = std::chrono::high_resolution_clock::now();
    duration = end_time - start_time;
    std::cout << "Time taken for voxel_downsample cuda: " << duration.count() << " ms."
              << std::endl;
  }
  // Downsample the frame to a voxel grid and print the downsampled frame
  start_time = std::chrono::high_resolution_clock::now();
  std::vector<tam::core::state::types::Point<tam::core::state::types::CUDA_ICP_EXT>>
    frame_downsampled =
      tam::core::state::voxel_downsample<tam::core::state::types::CUDA_ICP_EXT>(frame, 1.0);
  end_time = std::chrono::high_resolution_clock::now();

  // Calculate the duration
  duration = end_time - start_time;
  std::cout << "Time taken for voxel_downsample: " << duration.count() << " ms." << std::endl;

  // check if both methods output the same vector
  if (frame_downsampled.size() != frame_downsampled_cuda.size())
    std::cout << "Output of the CPU can CUDA version are not equal:\nSize CPU voxel Grid "
              << frame_downsampled.size() << "\nSize GPU voxel Grid "
              << frame_downsampled_cuda.size() << std::endl;
  return 0;
}
