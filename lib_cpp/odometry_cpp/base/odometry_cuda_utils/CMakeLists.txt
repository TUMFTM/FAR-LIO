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
