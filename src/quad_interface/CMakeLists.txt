cmake_minimum_required(VERSION 3.8)
project(quad_interface)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()


# DEPENDENCIES
find_package(ament_cmake REQUIRED)
# ros dependencies
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED) # TODO
# std interface
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_srvs REQUIRED)
# custom interface
find_package(raptor_interface REQUIRED)
# external dependencies
find_package(MAVSDK REQUIRED)


# TARGETS

# ref gen
add_executable(reference_generator src/reference_generator.cpp)
ament_target_dependencies(reference_generator 
  rclcpp 
  std_srvs 
  geometry_msgs
  raptor_interface)

# mav interface
add_executable(mav_interface src/mav_interface.cpp)
ament_target_dependencies(mav_interface 
  rclcpp 
  std_msgs 
  std_srvs 
  geometry_msgs 
  rclcpp_action # TODO
  raptor_interface)
target_include_directories(mav_interface PUBLIC include)
target_sources(mav_interface PUBLIC src/quad.cpp)
target_link_libraries(mav_interface MAVSDK::mavsdk)

#service
# add_executable(service_arm src/service_arm.cpp)
# ament_target_dependencies(service_arm rclcpp std_srvs)


# INSTALLATION
install(TARGETS
  reference_generator
  # service_arm
  mav_interface
  DESTINATION lib/${PROJECT_NAME})

# ament_export_dependencies(rosidl_default_runtime)
ament_package()