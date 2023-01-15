cmake_minimum_required(VERSION 3.13)

project(quad_actions)

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

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()
# Compile flags
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()


# TARGETS
# mav interface
add_executable(mav_interface src/mav_interface.cpp)
target_include_directories(mav_interface PUBLIC 
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
  "$<INSTALL_INTERFACE:include/${PROJECT_NAME}>")
target_sources(mav_interface PUBLIC src/quad.cpp)
ament_target_dependencies(mav_interface 
  rclcpp 
  std_msgs 
  std_srvs 
  geometry_msgs 
  raptor_interface)
target_link_libraries(mav_interface MAVSDK::mavsdk)

# ref gen
add_executable(reference_generator src/reference_generator.cpp)
ament_target_dependencies(reference_generator 
  rclcpp 
  std_srvs 
  geometry_msgs
  raptor_interface)


# INSTALLATION
install(TARGETS
  reference_generator
  mav_interface
  DESTINATION lib/${PROJECT_NAME})


ament_package()