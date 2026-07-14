set(PROJECT_NAME odometry_types)

add_library(${PROJECT_NAME} INTERFACE)

target_include_directories(${PROJECT_NAME}
    INTERFACE
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>
)

set(MODULE_DEPENDENCIES
  Eigen3::Eigen
  Sophus::Sophus
  tum_types_cpp::tum_types_cpp
)

target_link_libraries(${PROJECT_NAME} INTERFACE ${MODULE_DEPENDENCIES})
