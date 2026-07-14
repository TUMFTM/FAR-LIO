set(PROJECT_NAME model_handler)

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
  odometry_types
)

target_link_libraries(${PROJECT_NAME} INTERFACE ${MODULE_DEPENDENCIES})

# ==============================================================================
# UNIT TESTS
# ==============================================================================
if(BUILD_TESTING)
  set(TESTS
    unit_external_guess
    unit_constant_velocity
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
    external_guess
    constant_velocity
  )
  foreach(EXAMPLE ${EXAMPLES})
    add_executable(${EXAMPLE}_example example/${EXAMPLE}.cpp)
    target_link_libraries(${EXAMPLE}_example ${PROJECT_NAME})
  endforeach()
endif()
