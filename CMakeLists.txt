# Copyright(c) 2020 livoxtech limited.

cmake_minimum_required(VERSION 3.14)

include(cmake/version.cmake)
project(livox_ros_driver2_core VERSION ${LIVOX_ROS_DRIVER2_VERSION} LANGUAGES CXX)
message(STATUS "${PROJECT_NAME} version: ${LIVOX_ROS_DRIVER2_VERSION}")
execute_process(COMMAND git describe --always --dirty --abbrev=40
  WORKING_DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}"
  OUTPUT_VARIABLE LIVOX_BUILD_REVISION OUTPUT_STRIP_TRAILING_WHITESPACE ERROR_QUIET)
if(NOT LIVOX_BUILD_REVISION)
  set(LIVOX_BUILD_REVISION "unknown (source archive without Git metadata)")
endif()
file(WRITE "${CMAKE_CURRENT_BINARY_DIR}/driver_revision.txt" "${LIVOX_BUILD_REVISION}\n")
install(FILES "${CMAKE_CURRENT_BINARY_DIR}/driver_revision.txt" DESTINATION share/${PROJECT_NAME})

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
  src/lidar_state_recorder.cpp

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

option(BUILD_DOCS "Build the Doxygen deployment and developer manual" OFF)
if(BUILD_DOCS)
  find_package(Doxygen REQUIRED COMPONENTS dot)
  find_package(Python3 3.9 REQUIRED COMPONENTS Interpreter)
  set(LIVOX_SOURCE_DIR "${CMAKE_CURRENT_SOURCE_DIR}")
  set(DOXYGEN_OUTPUT_DIRECTORY "${CMAKE_CURRENT_BINARY_DIR}/docs")
  configure_file(
    "${CMAKE_CURRENT_SOURCE_DIR}/docs/Doxyfile.in"
    "${CMAKE_CURRENT_BINARY_DIR}/Doxyfile"
    @ONLY)
  add_custom_target(docs ALL
    COMMAND "${Python3_EXECUTABLE}" "${CMAKE_CURRENT_SOURCE_DIR}/docs/maintenance/check_links.py"
    COMMAND "${DOXYGEN_EXECUTABLE}" "${CMAKE_CURRENT_BINARY_DIR}/Doxyfile"
    COMMAND "${CMAKE_COMMAND}"
            "-DWARN_LOG=${CMAKE_CURRENT_BINARY_DIR}/doxygen-warnings.log"
            -P "${CMAKE_CURRENT_SOURCE_DIR}/docs/check_warnings.cmake"
    COMMAND "${Python3_EXECUTABLE}" "${CMAKE_CURRENT_SOURCE_DIR}/docs/maintenance/prepare_html.py"
            --html "${DOXYGEN_OUTPUT_DIRECTORY}/html"
    COMMAND "${Python3_EXECUTABLE}" "${CMAKE_CURRENT_SOURCE_DIR}/docs/maintenance/check_links.py"
            --html "${DOXYGEN_OUTPUT_DIRECTORY}/html"
    WORKING_DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}"
    COMMENT "Generating Livox driver deployment and developer manual"
    VERBATIM)
endif()

if(BUILD_TESTING)
  enable_testing()
  find_package(Python3 REQUIRED COMPONENTS Interpreter)
  add_test(NAME sany_ptp_guard_test
    COMMAND ${Python3_EXECUTABLE} ${CMAKE_CURRENT_SOURCE_DIR}/test/test_sany_ptp_guard.py)
  add_test(NAME collect_lidar_identity_test
    COMMAND ${Python3_EXECUTABLE} ${CMAKE_CURRENT_SOURCE_DIR}/test/test_collect_lidar_identity.py)
  add_test(NAME lidar_run_bundle_test
    COMMAND ${Python3_EXECUTABLE} ${CMAKE_CURRENT_SOURCE_DIR}/test/test_lidar_run_bundle.py)
  add_test(NAME diagnose_lidar_test
    COMMAND ${Python3_EXECUTABLE} ${CMAKE_CURRENT_SOURCE_DIR}/test/test_diagnose_lidar.py)
  add_executable(lidar_identity_test test/lidar_identity_test.cpp)
  target_include_directories(lidar_identity_test PRIVATE src ${LIVOX_LIDAR_SDK_INCLUDE_DIR})
  ament_target_dependencies(lidar_identity_test rclcpp)
  add_test(NAME lidar_identity_test COMMAND lidar_identity_test)
  add_test(NAME identity_collector_ros_test
    COMMAND ${Python3_EXECUTABLE} ${CMAKE_CURRENT_SOURCE_DIR}/test/test_identity_collector_ros.py
      --fixture $<TARGET_FILE:lidar_identity_test>)
  set_tests_properties(lidar_identity_test identity_collector_ros_test PROPERTIES
    TIMEOUT 60 RUN_SERIAL TRUE ENVIRONMENT "ROS_DOMAIN_ID=179;ROS_LOCALHOST_ONLY=1")
  # Cross-process discovery on WSL loopback is tested with CycloneDDS; production
  # middleware remains selected by the deployment environment.
  set_property(TEST identity_collector_ros_test APPEND PROPERTY ENVIRONMENT
    "RMW_IMPLEMENTATION=rmw_cyclonedds_cpp")
  add_executable(lidar_dtc_monitor_test test/lidar_dtc_monitor_test.cpp)
  target_include_directories(lidar_dtc_monitor_test PRIVATE src)
  add_test(NAME lidar_dtc_monitor_test COMMAND lidar_dtc_monitor_test)
  find_package(Threads REQUIRED)
  add_executable(lidar_state_recorder_test test/lidar_state_recorder_test.cpp src/lidar_state_recorder.cpp)
  target_include_directories(lidar_state_recorder_test PRIVATE src ../Livox-SDK2/3rdparty)
  target_link_libraries(lidar_state_recorder_test Threads::Threads)
  add_test(NAME lidar_state_recorder_test COMMAND lidar_state_recorder_test)
  set_tests_properties(lidar_state_recorder_test PROPERTIES TIMEOUT 20)
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

# Optional host-service deployment tools; installing the ROS package does not
# change systemd, Chrony, RTC, or running sensors.
install(PROGRAMS
  scripts/sany_ptp_guard.py
  scripts/collect_lidar_identity.py
  scripts/run_lidar_bundle.py
  scripts/run_lidar.sh
  scripts/lidar_bundle_check.py
  scripts/diagnose_lidar.py
  scripts/install_sany_ptp_guard.sh
  DESTINATION lib/${PROJECT_NAME}
)


if(BUILD_TESTING)
  add_executable(heartbeat_dtc_test test/heartbeat_dtc_test.cpp src/functional_safety_heartbeat.cpp)
  target_include_directories(heartbeat_dtc_test PRIVATE src)
  ament_target_dependencies(heartbeat_dtc_test rclcpp diagnostic_monitor_interfaces)
  add_test(NAME heartbeat_dtc_test COMMAND heartbeat_dtc_test)
  add_executable(driver_dtc_reporting_test test/driver_dtc_reporting_test.cpp src/functional_safety_heartbeat.cpp)
  target_include_directories(driver_dtc_reporting_test PRIVATE src)
  ament_target_dependencies(driver_dtc_reporting_test rclcpp diagnostic_monitor_interfaces livox_ros_driver2)
  add_test(NAME driver_dtc_reporting_test COMMAND driver_dtc_reporting_test)
endif()

ament_auto_package()
