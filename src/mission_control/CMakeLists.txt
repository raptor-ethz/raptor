cmake_minimum_required(VERSION 3.13)

project(mission_control)


# DEPENDENCIES
# ros default
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(std_msgs REQUIRED)
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
# takeoff_test
add_executable(takeoff_test src/missions/takeoff_test.cpp)
target_include_directories(takeoff_test PUBLIC 
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
  "$<INSTALL_INTERFACE:include/${PROJECT_NAME}>")
target_sources(takeoff_test PUBLIC 
  src/mission_control.cpp
  src/actions/arm.cpp
  src/actions/land.cpp
  src/actions/takeoff.cpp
  src/actions/go_to_pos.cpp)
ament_target_dependencies(takeoff_test 
  rclcpp 
  rclcpp_action
  raptor_interface)

# square
add_executable(square src/missions/square.cpp)
target_include_directories(square PUBLIC 
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
  "$<INSTALL_INTERFACE:include/${PROJECT_NAME}>")
target_sources(square PUBLIC 
  src/mission_control.cpp
  src/actions/arm.cpp
  src/actions/land.cpp
  src/actions/takeoff.cpp
  src/actions/go_to_pos.cpp)
ament_target_dependencies(square 
  rclcpp 
  rclcpp_action
  raptor_interface)

# swoop
add_executable(swoop src/missions/swoop.cpp)
target_include_directories(swoop PUBLIC 
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
  "$<INSTALL_INTERFACE:include/${PROJECT_NAME}>")
target_sources(swoop PUBLIC 
  src/mission_control.cpp
  src/actions/arm.cpp
  src/actions/land.cpp
  src/actions/takeoff.cpp
  src/actions/go_to_pos.cpp)
ament_target_dependencies(swoop 
  rclcpp 
  rclcpp_action
  raptor_interface)

# grasp
add_executable(grasp src/missions/grasp.cpp)
target_include_directories(grasp PUBLIC 
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
  "$<INSTALL_INTERFACE:include/${PROJECT_NAME}>")
target_sources(grasp PUBLIC 
  src/mission_control.cpp
  src/actions/arm.cpp
  src/actions/land.cpp
  src/actions/takeoff.cpp
  src/actions/go_to_pos.cpp)
ament_target_dependencies(grasp 
  rclcpp 
  rclcpp_action
  raptor_interface)

# gripper_test
add_executable(gripper_test src/missions/gripper_test.cpp)
target_include_directories(gripper_test PUBLIC 
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
  "$<INSTALL_INTERFACE:include/${PROJECT_NAME}>")
target_sources(gripper_test PUBLIC 
  src/mission_control.cpp
  src/actions/arm.cpp
  src/actions/land.cpp
  src/actions/takeoff.cpp
  src/actions/go_to_pos.cpp
  src/actions/set_gripper.cpp)
ament_target_dependencies(gripper_test 
  rclcpp 
  rclcpp_action
  raptor_interface)

# acceleration_test
add_executable(acceleration_test src/missions/acceleration_test.cpp)
target_include_directories(acceleration_test PUBLIC 
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
  "$<INSTALL_INTERFACE:include/${PROJECT_NAME}>")
target_sources(acceleration_test PUBLIC 
  src/mission_control.cpp
  src/actions/arm.cpp
  src/actions/land.cpp
  src/actions/takeoff.cpp
  src/actions/go_to_pos.cpp
  src/actions/hover_acc.cpp)
ament_target_dependencies(acceleration_test 
  rclcpp 
  rclcpp_action
  raptor_interface)

# dev_publisher
add_executable(dev_publisher src/missions/dev_publisher.cpp)
target_include_directories(dev_publisher PUBLIC 
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
  "$<INSTALL_INTERFACE:include/${PROJECT_NAME}>")
target_sources(dev_publisher PUBLIC src/missions/dev_publisher.cpp)
ament_target_dependencies(dev_publisher 
  rclcpp
  std_msgs)


# INSTALLATION
install(TARGETS
  takeoff_test
  square
  swoop
  grasp
  gripper_test
  acceleration_test
  dev_publisher
  DESTINATION lib/${PROJECT_NAME})


ament_package()