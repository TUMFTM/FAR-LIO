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
set(PROJECT_NAME odometry_pipeline)

# ==============================================================================
# Library
# ==============================================================================
set(SOURCES
  src/odometry_pipeline.cpp
)
if(BUILD_CUDA)
  list(APPEND SOURCES src/odometry_pipeline.cu)
endif()

add_library(${PROJECT_NAME} SHARED ${SOURCES})

# Public include dirs for consumers
target_include_directories(${PROJECT_NAME}
    PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>
)

set(MODULE_DEPENDENCIES
  covariance_handler
  diagnostic_handler
  distortion_handler
  map_handler
  model_handler
  preprocessing_handler
  registration_handler
  threshold_handler
  velocity_handler
)

target_link_libraries(${PROJECT_NAME} PUBLIC ${MODULE_DEPENDENCIES})

set_cuda_architecture(${PROJECT_NAME})

# Install the library
install(TARGETS ${PROJECT_NAME}
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

# ==============================================================================
# UNIT TESTS
# ==============================================================================
if(BUILD_TESTING)
  set(TESTS
    unit_odometry_pipeline
  )
  include(GoogleTest)
  foreach(TEST ${TESTS})
    add_executable(${TEST} test/${TEST}.cpp)
    target_link_libraries(${TEST} ${PROJECT_NAME} GTest::gtest_main)
    gtest_discover_tests(${TEST})
  endforeach()
endif()

# ==============================================================================
# EXAMPLES
# ==============================================================================
if(BUILD_ODOMETRY_EXAMPLES)
  set(EXAMPLES
    odometry_pipeline_example
  )
  foreach(EXAMPLE ${EXAMPLES})
    add_executable(${EXAMPLE} example/${EXAMPLE}.cpp)
    target_link_libraries(${EXAMPLE} PRIVATE ${PROJECT_NAME})
  endforeach()
endif()
