# Copyright 2026 Maximilian Leitenstern, Marcel Weinmann
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
set(PROJECT_NAME odometry_cuda_utils)

add_library(${PROJECT_NAME} INTERFACE)

target_include_directories(${PROJECT_NAME}
    INTERFACE
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>
)

# GTest is required by the CHECK_CUDA_ERROR macro in cuda_utils.cuh
set(MODULE_DEPENDENCIES
  Eigen3::Eigen
  Sophus::Sophus
  odometry_types
  odometry_utils
  GTest::gtest_main
  cuco
)

target_link_libraries(${PROJECT_NAME} INTERFACE ${MODULE_DEPENDENCIES})

set_cuda_architecture(${PROJECT_NAME})
