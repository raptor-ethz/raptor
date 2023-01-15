cmake_minimum_required(VERSION 3.8)

project(vicon)


# DEPENDENCIES
# ros default
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
# ros custom
find_package(raptor_interface REQUIRED)


# Compiler options
# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()
# Compile flags
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()


# TARGETS
# vicon datastream
add_subdirectory(vicon_datastream)

# vicon publisher
add_executable(vicon_publisher src/vicon_publisher.cpp)
target_include_directories(vicon_publisher PUBLIC include)
ament_target_dependencies(vicon_publisher 
  rclcpp
  raptor_interface)
target_link_libraries(vicon_publisher ViconDataStreamSDK_lib)


# INSTALLATION
install(TARGETS
  vicon_publisher
  DESTINATION lib/${PROJECT_NAME})


ament_package()