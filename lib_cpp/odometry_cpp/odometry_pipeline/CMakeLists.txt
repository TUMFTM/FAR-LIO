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
