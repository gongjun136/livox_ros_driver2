# Copyright(c) 2020 livoxtech limited.

cmake_minimum_required(VERSION 3.14)

include(cmake/version.cmake)
project(livox_ros_driver2_core VERSION ${LIVOX_ROS_DRIVER2_VERSION} LANGUAGES CXX)
message(STATUS "${PROJECT_NAME} version: ${LIVOX_ROS_DRIVER2_VERSION}")

if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
  set(CMAKE_BUILD_TYPE Release CACHE STRING "Build type" FORCE)
endif()

if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

list(INSERT CMAKE_MODULE_PATH 0 "${PROJECT_SOURCE_DIR}/cmake/modules")

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -Wno-unused-parameter $<$<CONFIG:Release>:-O3>)
endif()

add_definitions(-DBUILDING_ROS2)

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()
find_package(PCL REQUIRED)
find_package(diagnostic_monitor_interfaces REQUIRED)
find_package(livox_ros_driver2 REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)

find_package(PkgConfig)
pkg_check_modules(APR apr-1)
if(APR_FOUND)
  message(STATUS "APR include directories: ${APR_INCLUDE_DIRS}")
  message(STATUS "APR libraries: ${APR_LIBRARIES}")
endif()
pkg_check_modules(ZSTD REQUIRED libzstd)

# Find Livox SDK2 either in /usr/local (the upstream install layout) or in
# this colcon workspace's isolated install directory.
get_filename_component(COLCON_BUILD_ROOT "${CMAKE_BINARY_DIR}" DIRECTORY)
get_filename_component(COLCON_WORKSPACE_ROOT "${COLCON_BUILD_ROOT}" DIRECTORY)
set(LIVOX_SDK2_ROOT "" CACHE PATH "Livox SDK2 installation prefix")
find_library(LIVOX_LIDAR_SDK_LIBRARY
  NAMES livox_lidar_sdk_shared
  HINTS
    "${LIVOX_SDK2_ROOT}"
    "${COLCON_WORKSPACE_ROOT}/install/livox_sdk2"
    "/usr/local"
  PATH_SUFFIXES lib
  REQUIRED)

find_path(LIVOX_LIDAR_SDK_INCLUDE_DIR
  NAMES "livox_lidar_api.h" "livox_lidar_def.h"
  HINTS
    "${LIVOX_SDK2_ROOT}"
    "${COLCON_WORKSPACE_ROOT}/install/livox_sdk2"
    "/usr/local"
  PATH_SUFFIXES include
  REQUIRED)

link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

ament_auto_add_library(${PROJECT_NAME} SHARED
  src/livox_ros_driver2.cpp
  src/lddc.cpp
  src/driver_node.cpp
  src/functional_safety_heartbeat.cpp
  src/lds.cpp
  src/lds_lidar.cpp

  src/comm/comm.cpp
  src/comm/ldq.cpp
  src/comm/semaphore.cpp
  src/comm/lidar_imu_data_queue.cpp
  src/comm/cache_index.cpp
  src/comm/pub_handler.cpp

  src/parse_cfg_file/parse_cfg_file.cpp
  src/parse_cfg_file/parse_livox_lidar_cfg.cpp

  src/call_back/lidar_common_callback.cpp
  src/call_back/livox_lidar_callback.cpp
)

ament_target_dependencies(${PROJECT_NAME}
  diagnostic_monitor_interfaces
  livox_ros_driver2
)

target_include_directories(${PROJECT_NAME} PUBLIC
  ${PCL_INCLUDE_DIRS}
  ${APR_INCLUDE_DIRS}
  ${LIVOX_LIDAR_SDK_INCLUDE_DIR}
  ../Livox-SDK2/3rdparty
  src
)

target_link_libraries(${PROJECT_NAME}
  ${LIVOX_LIDAR_SDK_LIBRARY}
  ${PPT_LIBRARY}
  ${Boost_LIBRARY}
  ${PCL_LIBRARIES}
  ${APR_LIBRARIES}
)

rclcpp_components_register_node(${PROJECT_NAME}
  PLUGIN "livox_ros::DriverNode"
  EXECUTABLE livox_ros_driver2_node
)

add_executable(pointcloud_zstd_compressor
  src/compression/pointcloud_zstd_compressor.cpp
)
ament_target_dependencies(pointcloud_zstd_compressor
  livox_ros_driver2
  rclcpp
  sensor_msgs
  std_msgs
)
target_include_directories(pointcloud_zstd_compressor PRIVATE
  ${ZSTD_INCLUDE_DIRS}
)
target_link_libraries(pointcloud_zstd_compressor
  ${ZSTD_LIBRARIES}
)

add_executable(pointcloud_zstd_decompressor
  src/compression/pointcloud_zstd_decompressor.cpp
)
ament_target_dependencies(pointcloud_zstd_decompressor
  livox_ros_driver2
  rclcpp
  sensor_msgs
  std_msgs
)
target_include_directories(pointcloud_zstd_decompressor PRIVATE
  ${ZSTD_INCLUDE_DIRS}
)
target_link_libraries(pointcloud_zstd_decompressor
  ${ZSTD_LIBRARIES}
)

install(TARGETS
  pointcloud_zstd_compressor
  pointcloud_zstd_decompressor
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

install(DIRECTORY
  launch_ROS2
  config
  docs
  DESTINATION share/${PROJECT_NAME}
)

install(PROGRAMS
  scripts/record_3lidars_mcap.sh
  DESTINATION lib/${PROJECT_NAME}
)

install(PROGRAMS
  scripts/record_3lidars_mcap.sh
  DESTINATION lib/${PROJECT_NAME}
  RENAME record_lidars_mcap.sh
)

install(PROGRAMS
  scripts/record_lidars_cameras_mcap.sh
  DESTINATION lib/${PROJECT_NAME}
)

install(PROGRAMS
  scripts/record_compressed_sensors_mcap.sh
  DESTINATION lib/${PROJECT_NAME}
)

ament_auto_package()
