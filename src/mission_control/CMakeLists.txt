cmake_minimum_required(VERSION 3.13)

project(mission_control)


# DEPENDENCIES
# ros default
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
# ros custom
find_package(raptor_interface REQUIRED)


# Compiler options
# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
# Compile flags
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()


# TARGETS
add_executable(takeoff_test src/missions/takeoff_test.cpp)
target_include_directories(takeoff_test PUBLIC 
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
  "$<INSTALL_INTERFACE:include/${PROJECT_NAME}>")
target_sources(takeoff_test PUBLIC 
  src/mission_control.cpp
  src/actions/arm.cpp
  src/actions/takeoff.cpp)
ament_target_dependencies(takeoff_test 
  rclcpp 
  rclcpp_action
  raptor_interface)


# INSTALLATION
install(TARGETS
  takeoff_test
  DESTINATION lib/${PROJECT_NAME})


ament_package()