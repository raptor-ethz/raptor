cmake_minimum_required(VERSION 3.13)

project(quad_control)


# DEPENDENCIES
# ros default
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_srvs REQUIRED)
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
add_executable(quad_control src/quad_control_app.cpp)
target_include_directories(quad_control PUBLIC 
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
  "$<INSTALL_INTERFACE:include/${PROJECT_NAME}>")
target_sources(quad_control PUBLIC 
  src/quad.cpp
  src/mavsdk_wrapper.cpp)
ament_target_dependencies(quad_control 
  rclcpp 
  raptor_interface)
target_link_libraries(quad_control MAVSDK::mavsdk)

# ref gen
add_executable(reference_generator src/reference_generator.cpp)
ament_target_dependencies(reference_generator 
  rclcpp 
  std_srvs 
  geometry_msgs
  raptor_interface)


# INSTALLATION
install(TARGETS
  quad_control
  reference_generator
  DESTINATION lib/${PROJECT_NAME})


ament_package()