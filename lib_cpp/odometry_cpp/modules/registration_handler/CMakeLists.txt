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
set(PROJECT_NAME registration_handler)

add_library(${PROJECT_NAME} INTERFACE)

target_include_directories(${PROJECT_NAME}
    INTERFACE
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>
)

set(MODULE_DEPENDENCIES
  Eigen3::Eigen
  Sophus::Sophus
  odometry_base
  odometry_utils
  odometry_types
  map_handler
  robust_kernel
)
if(BUILD_CUDA)
  list(APPEND MODULE_DEPENDENCIES odometry_cuda_utils)
endif()

target_link_libraries(${PROJECT_NAME} INTERFACE ${MODULE_DEPENDENCIES})

set_cuda_architecture(${PROJECT_NAME})

# ==============================================================================
# UNIT TESTS
# ==============================================================================
if(BUILD_TESTING)
  set(TESTS
    unit_icp
    unit_gicp
  )
  set(TESTS_CUDA
    unit_icp
    unit_gicp
  )
  include(GoogleTest)
  foreach(TEST ${TESTS})
    add_executable(${TEST} test/${TEST}.cpp)
    target_link_libraries(${TEST} ${PROJECT_NAME} GTest::gtest_main)
    gtest_discover_tests(${TEST})
  endforeach()
  if(BUILD_CUDA)
    foreach(TEST ${TESTS_CUDA})
      add_executable(${TEST}_cuda test/${TEST}.cu)
      target_link_libraries(${TEST}_cuda ${PROJECT_NAME} GTest::gtest_main)
      set_cuda_architecture(${TEST}_cuda)
      gtest_discover_tests(${TEST}_cuda)
    endforeach()
  endif()
endif()

# ==============================================================================
# EXAMPLES
# ==============================================================================
if(BUILD_ODOMETRY_EXAMPLES)
  set(EXAMPLES
    icp
    gicp
  )
  set(EXAMPLES_CUDA
    icp
    gicp
  )
  foreach(EXAMPLE ${EXAMPLES})
    add_executable(${EXAMPLE}_example example/${EXAMPLE}.cpp)
    target_link_libraries(${EXAMPLE}_example ${PROJECT_NAME})
  endforeach()
  if(BUILD_CUDA)
    foreach(EXAMPLE ${EXAMPLES_CUDA})
      add_executable(${EXAMPLE}_example_cuda example/${EXAMPLE}.cu)
      target_link_libraries(${EXAMPLE}_example_cuda ${PROJECT_NAME})
      set_cuda_architecture(${EXAMPLE}_example_cuda)
    endforeach()
  endif()
endif()
