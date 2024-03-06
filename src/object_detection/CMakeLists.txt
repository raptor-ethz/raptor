cmake_minimum_required(VERSION 3.13)

project(object_detection)


# DEPENDENCIES
# ros default
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
# find_package(cv_bridge REQUIRED)
# ros custom
find_package(raptor_interface REQUIRED)
# external
find_package(MAVSDK REQUIRED)


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
# quad control
install(PROGRAMS
  src/detection.py
  src/frame_transformations.py
  src/pointcloud.py
  src/logger.py
  src/utils.py
  src/streamer_sender.py
  src/streamer_receiver.py
  src/visualizer.py
  src/realsense.py
  src/pointcloud_tests.py
  src/kalman.py
  DESTINATION lib/${PROJECT_NAME}
)

# INSTALLATION
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME})


ament_package()