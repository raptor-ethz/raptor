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
# vicon_datastream
add_subdirectory(vicon_datastream)

# vicon
add_executable(vicon src/vicon.cpp)
target_include_directories(vicon PUBLIC include)
ament_target_dependencies(vicon 
  rclcpp
  raptor_interface)
target_link_libraries(vicon ViconDataStreamSDK_lib)


# INSTALLATION
install(TARGETS
  vicon
  DESTINATION lib/${PROJECT_NAME})


ament_package()