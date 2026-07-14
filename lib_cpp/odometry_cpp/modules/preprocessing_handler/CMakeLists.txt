set(PROJECT_NAME preprocessing_handler)

add_library(${PROJECT_NAME} INTERFACE)

target_include_directories(${PROJECT_NAME}
    INTERFACE
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<BUILD_INTERFACE:${robin_map_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>
)

set(MODULE_DEPENDENCIES
  Eigen3::Eigen
  odometry_base
  odometry_utils
  odometry_types
  voxel_tools
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
    unit_lidar_preprocessing
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
  )
  # Add .cu examples here to build them with BUILD_CUDA
  set(EXAMPLES_CUDA)
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
