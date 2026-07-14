set(PROJECT_NAME odometry_utils)

add_library(${PROJECT_NAME} INTERFACE)

target_include_directories(${PROJECT_NAME}
    INTERFACE
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>
)

set(MODULE_DEPENDENCIES
  odometry_types
)
if(VISUALIZATION)
  list(APPEND MODULE_DEPENDENCIES rerun_sdk)
endif()

target_link_libraries(${PROJECT_NAME} INTERFACE ${MODULE_DEPENDENCIES})
