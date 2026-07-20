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
cmake_minimum_required(VERSION 3.25.2)
set(PROJECT_NAME odometry_cpp)
if (BUILD_CUDA)
  project(${PROJECT_NAME} LANGUAGES C CXX CUDA)
else()
  project(${PROJECT_NAME} LANGUAGES C CXX)
endif()

# ==============================================================================
# Options
# ==============================================================================
option(BUILD_CUDA "Build CUDA code" OFF)
option(VISUALIZATION "Build visualization library using rerun" OFF)
option(BUILD_ODOMETRY_EXAMPLES "Build example executables" ON)
# NOTE: named with ODOMETRY prefix to avoid clashing with the BUILD_EXAMPLES option of
# fetched dependencies (e.g. cuCollections)
# BUILD_TESTING is predefined when built via ament/colcon; default it for standalone builds
option(BUILD_TESTING "Build unit tests" ON)

# ==============================================================================
# Dependencies
# ==============================================================================
# C++ standard
find_package(Eigen3 3.4 REQUIRED)

# TAM Common packages
find_package(param_management_cpp REQUIRED)
find_package(tsl_logger_cpp REQUIRED)

# Fetch additional libraries
include(FetchContent)
# Load tsl robin map (header only library)
FetchContent_Declare(robin_map URL https://github.com/Tessil/robin-map/archive/refs/tags/v1.4.0.tar.gz)
# Google Test Library (for unit tests)
FetchContent_Declare(googletest URL https://github.com/google/googletest/archive/03597a01ee50ed33e9dfd640b249b4be3799d395.zip)
# Fetch Sophus
set(BUILD_SOPHUS_TESTS OFF CACHE BOOL "Don't build Sophus tests")
FetchContent_Declare(Sophus URL https://github.com/strasdat/Sophus/archive/refs/tags/1.24.6.tar.gz)
# TBB multithreading library
set(TBBMALLOC_BUILD OFF CACHE BOOL "Don't build TBB malloc")
set(TBB_EXAMPLES OFF CACHE BOOL "Don't build TBB examples")
set(TBB_STRICT OFF CACHE BOOL "Don't treat TBB warnings as errors")
set(TBB_TEST OFF CACHE BOOL "Don't build TBB tests")
FetchContent_Declare(tbb URL https://github.com/uxlfoundation/oneTBB/archive/refs/tags/v2022.0.0.tar.gz)

# Make all libraries available
FetchContent_MakeAvailable(robin_map googletest Sophus tbb)

# Fetch rerun if configured
if(VISUALIZATION)
  set(BUILD_SHARED_LIBS ON CACHE BOOL "Build rerun_sdk as a shared library" FORCE)
  FetchContent_Declare(rerun_sdk URL https://github.com/rerun-io/rerun/releases/latest/download/rerun_cpp_sdk.zip)
  FetchContent_MakeAvailable(rerun_sdk)
endif()

# Fetch CUDA dependencies if configured
if (BUILD_CUDA)
  # Fetch CuCollections
  # Set build options
  set(BUILD_TESTS OFF CACHE BOOL "Don't build cuCollections tests")
  set(BUILD_BENCHMARKS OFF CACHE BOOL "Don't build cuCollections benchmarks")
  set(BUILD_EXAMPLES OFF CACHE BOOL "Don't build cuCollections examples")
  FetchContent_Declare(
    cuCollections
    GIT_REPOSITORY https://github.com/NVIDIA/cuCollections.git
    GIT_TAG 67110a53135ffdd43814794c6c39751f8d74e080
  )
  FetchContent_MakeAvailable(cuCollections)
endif()

# ==============================================================================
# Compiler settings
# ==============================================================================
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
set(CMAKE_POSITION_INDEPENDENT_CODE ON)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Add compile definition for visualization
if(VISUALIZATION)
    add_compile_definitions(USE_VISUALIZATION)
endif()

# Set C and CXX standard
set(CMAKE_C_STANDARD 99)
set(CMAKE_CXX_STANDARD 20)

# Set CUDA standards
if (BUILD_CUDA)
  add_compile_definitions(BUILD_CUDA)
  # Set CUDA standard to C++20
  set(CMAKE_CUDA_STANDARD 20)
  # Set CUDA compiler flags
  set(CMAKE_CUDA_FLAGS "-std=c++20")
  set(CMAKE_CUDA_FLAGS "${CMAKE_CUDA_FLAGS} --expt-extended-lambda")
  set(CMAKE_CUDA_FLAGS "${CMAKE_CUDA_FLAGS} --expt-relaxed-constexpr")
endif()

# Function to set CUDA architecture
function(set_cuda_architecture target)
  if (BUILD_CUDA)
    if(NOT DEFINED CUDA_ARCHITECTURES)
      message(FATAL_ERROR "CUDA_ARCHITECTURES is not defined. Please set it: -DCUDA_ARCHITECTURES=80")
    endif()
    set_target_properties(${target} PROPERTIES CUDA_ARCHITECTURES ${CUDA_ARCHITECTURES})
  endif()
endfunction()


# Enable unit testing
enable_testing()

# ==============================================================================
# Libraries
# ==============================================================================
# Types for odometry libraries containing containers and data structures
add_subdirectory(base/odometry_types)
# Base odometry library
add_subdirectory(base/odometry_base)
# Utils library
add_subdirectory(base/odometry_utils)
# Utils library
if (BUILD_CUDA)
  add_subdirectory(base/odometry_cuda_utils)
endif()
# Covariance handler library
add_subdirectory(modules/covariance_handler)
# Diagnostic status library
add_subdirectory(modules/diagnostic_handler)
# Distortion handler library
add_subdirectory(modules/distortion_handler)
# Map handler library
add_subdirectory(modules/map_handler)
# Prediction model library
add_subdirectory(modules/model_handler)
# Preprocessing handler library
add_subdirectory(modules/preprocessing_handler)
# Robust kernel library
add_subdirectory(modules/robust_kernel)
# Correspondence threshold library
add_subdirectory(modules/threshold_handler)
# Registration handler library
add_subdirectory(modules/registration_handler)
# Velocity handler library
add_subdirectory(modules/velocity_handler)
# Voxel tools library
add_subdirectory(modules/voxel_tools)
# Pipeline library
add_subdirectory(odometry_pipeline)
