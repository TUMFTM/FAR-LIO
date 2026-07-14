cmake_minimum_required(VERSION 3.25.2)

# CONFIGURE PKG HERE
# ==========================================
set(PROJECT_NAME tam_odometry)
set(NODE_LIBRARY_PATHS src/tam_odometry.cpp)
set(EXECUTABLE_NAMES tam_icp_ext tam_icp_cv tam_gicp_ext)
set(EXECUTABLE_NAMES_CUDA tam_cuda_icp_ext tam_cuda_gicp_ext)
set(COMPONENT_NAMES ICP_EXT ICP_CV GICP_EXT)
set(COMPONENT_NAMES_CUDA CUDA_ICP_EXT CUDA_GICP_EXT)
set(NODE_DEPENDENCIES
  rclcpp
  rclcpp_components
  tf2_ros
  tf2_geometry_msgs
  geometry_msgs
  sensor_msgs
  nav_msgs
  pcl_conversions
  PCL
  ros2_watchdog_cpp
  tum_map_msgs
  tum_ros_helpers_cpp
  tsl_ros2_publisher_cpp
)
# ============================================
# END OF CONFIGURATION
if(BUILD_CUDA)
  project(${PROJECT_NAME} LANGUAGES C CXX CUDA)
  set(CMAKE_CUDA_STANDARD 20)
else()
  project(${PROJECT_NAME})
endif()

set(CMAKE_C_STANDARD 99)
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
foreach(pkg_name ${NODE_DEPENDENCIES})
  find_package(${pkg_name} REQUIRED QUIET)
endforeach()

# Subdirectory for cpp library
if(EXISTS ${CMAKE_CURRENT_SOURCE_DIR}/../../lib_cpp/odometry_cpp/)
  add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/../../lib_cpp/odometry_cpp ${CMAKE_CURRENT_BINARY_DIR}/tam_odometry)
else()
  message(FATAL_ERROR "odometry_cpp directory not found by cmake")
endif()

add_library(${PROJECT_NAME} SHARED ${NODE_LIBRARY_PATHS})

if(BUILD_CUDA)
  target_compile_definitions(${PROJECT_NAME} PRIVATE BUILD_CUDA)
endif()

target_compile_features(${PROJECT_NAME} PUBLIC c_std_99 cxx_std_20)
target_include_directories(${PROJECT_NAME} 
  PRIVATE  
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<BUILD_INTERFACE:${PCL_INCLUDE_DIRS}>
    $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(${PROJECT_NAME} ${NODE_DEPENDENCIES})

target_link_libraries(${PROJECT_NAME} odometry_pipeline ${PCL_LIBRARIES})

# CPU nodes
foreach(COMPONENT_NAME EXECUTABLE_NAME IN ZIP_LISTS COMPONENT_NAMES EXECUTABLE_NAMES)
  # Register component and executable for each node
  rclcpp_components_register_node(${PROJECT_NAME}
      PLUGIN ${PROJECT_NAME}::${COMPONENT_NAME}
      EXECUTABLE ${EXECUTABLE_NAME}
  )
endforeach()

# CUDA nodes
if (BUILD_CUDA)
  foreach(COMPONENT_NAME EXECUTABLE_NAME IN ZIP_LISTS COMPONENT_NAMES_CUDA EXECUTABLE_NAMES_CUDA)
    # Register component and executable for each node
    rclcpp_components_register_node(${PROJECT_NAME}
        PLUGIN ${PROJECT_NAME}::${COMPONENT_NAME}
        EXECUTABLE ${EXECUTABLE_NAME}
    )
  endforeach()
endif()


install(TARGETS ${PROJECT_NAME}
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
  )

ament_package()
