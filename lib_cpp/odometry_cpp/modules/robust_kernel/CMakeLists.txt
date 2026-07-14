set(PROJECT_NAME robust_kernel)

add_library(${PROJECT_NAME} INTERFACE)

target_include_directories(${PROJECT_NAME}
    INTERFACE
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>
)

set(MODULE_DEPENDENCIES
  odometry_types
)

target_link_libraries(${PROJECT_NAME} INTERFACE ${MODULE_DEPENDENCIES})

set_cuda_architecture(${PROJECT_NAME})

# ==============================================================================
# UNIT TESTS
# ==============================================================================
if(BUILD_TESTING)
  set(TESTS
    unit_robust_kernel
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
    robust_kernel
  )
  foreach(EXAMPLE ${EXAMPLES})
    add_executable(${EXAMPLE}_example example/${EXAMPLE}.cpp)
    target_link_libraries(${EXAMPLE}_example ${PROJECT_NAME})
  endforeach()
endif()
