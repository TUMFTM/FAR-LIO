# Copyright 2024 Marcel Weinmann, Maximilian Leitenstern
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
cmake_minimum_required(VERSION 3.12)

# CONFIGURE PKG HERE
# ==========================================
set(PROJECT_NAME tam_state_estimation_node)
set(COMPONENT_PATHS 
    components/state_estimation_node.cpp
    )
set(COMPONENT_NAMES
    ${PROJECT_NAME}::StateEstimation2DEKFKin
    ${PROJECT_NAME}::StateEstimation2DEKFNh
    ${PROJECT_NAME}::StateEstimation3DEKFKin
    ${PROJECT_NAME}::StateEstimation3DEKFNh
    ${PROJECT_NAME}::StateEstimation3DEKFStm
    )
set(EXECUTABLE_NAMES
    tam_state_estimation_node_ekf_2d_kin
    tam_state_estimation_node_ekf_2d_nh
    tam_state_estimation_node_ekf_3d_kin
    tam_state_estimation_node_ekf_3d_nh
    tam_state_estimation_node_ekf_3d_stm
    )

set(NODE_DEPENDENCIES
  rclcpp
  rclcpp_components
  tum_types_cpp
  tum_helpers_cpp
  tum_ros_helpers_cpp
  param_management_cpp
  param_management_ros2_integration_cpp
  tf2
  tf2_ros
  tf2_geometry_msgs
  nav_msgs 
  tum_msgs
  std_msgs
  sensor_msgs
  diagnostic_msgs
  autoware_auto_vehicle_msgs
  tam_state_estimation
  state_estimation_base
  state_estimation_constants
  tum_type_conversions_ros_cpp
  ros2_watchdog_cpp
  tsl_ros2_publisher_cpp
)

# ============================================
# END OF CONFIGURATION
project(${PROJECT_NAME})

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake)
foreach(pkg_name ${NODE_DEPENDENCIES})
  find_package(${pkg_name} REQUIRED)
endforeach()

# section: BUILD LIBRARY TARGET TO IMPORT NODE CLASS WITHIN OTHER PKGS
# ====================================================================
add_library(${PROJECT_NAME} SHARED ${COMPONENT_PATHS})

target_include_directories(${PROJECT_NAME}
    PUBLIC 
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>
)

target_compile_features(${PROJECT_NAME} PUBLIC c_std_99 cxx_std_20)

# section: REGISTER COMPONENTS AND EXECUTABLES
# ====================================================================
foreach(COMPONENT_NAME EXECUTABLE_NAME IN ZIP_LISTS COMPONENT_NAMES EXECUTABLE_NAMES)
  # Register component and executable for each node
  rclcpp_components_register_node(
    ${PROJECT_NAME}
      PLUGIN ${COMPONENT_NAME}
      EXECUTABLE ${EXECUTABLE_NAME}
  )
endforeach()

install(
  DIRECTORY include/
  DESTINATION include
)
install(
  TARGETS ${PROJECT_NAME}
  EXPORT export_${PROJECT_NAME}
  DESTINATION lib/${PROJECT_NAME}
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
)
ament_target_dependencies(${PROJECT_NAME} ${NODE_DEPENDENCIES})
ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET) # so client can also call target_link_libraries(template_lib_cpp::template_lib_cpp)

# section: GENERAL
# ====================================================================
ament_export_dependencies(rclcpp_components ${NODE_DEPENDENCIES}) # other dependencies so client does not need to call find_package on them
ament_package()
