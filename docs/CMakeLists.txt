cmake_minimum_required(VERSION 3.16)
project(livox_ros_driver2_docs NONE)

# Standalone manual build; ROS, the Livox SDK and a C++ compiler are not required.
find_package(Doxygen REQUIRED COMPONENTS dot)
get_filename_component(LIVOX_SOURCE_DIR "${CMAKE_CURRENT_SOURCE_DIR}/.." ABSOLUTE)
set(DOXYGEN_OUTPUT_DIRECTORY "${CMAKE_CURRENT_BINARY_DIR}/docs")
configure_file(Doxyfile.in "${CMAKE_CURRENT_BINARY_DIR}/Doxyfile" @ONLY)
add_custom_target(docs ALL
  COMMAND "${DOXYGEN_EXECUTABLE}" "${CMAKE_CURRENT_BINARY_DIR}/Doxyfile"
  COMMAND "${CMAKE_COMMAND}"
          "-DWARN_LOG=${CMAKE_CURRENT_BINARY_DIR}/doxygen-warnings.log"
          -P "${CMAKE_CURRENT_SOURCE_DIR}/check_warnings.cmake"
  WORKING_DIRECTORY "${LIVOX_SOURCE_DIR}"
  COMMENT "Generating Livox driver deployment and developer manual"
  VERBATIM)
