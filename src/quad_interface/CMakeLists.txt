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
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_srvs REQUIRED)
# find_package(builtin_interfaces REQUIRED)
# find_package(rosidl_default_generators REQUIRED)
# external dependencies
find_package(MAVSDK REQUIRED)


# INTERFACES
# rosidl_generate_interfaces(${PROJECT_NAME}
#   "srv/RequestArm.srv"
#   DEPENDENCIES builtin_interfaces
# )


# TARGETS

#talker
# add_executable(talker src/talker_node.cpp)
# ament_target_dependencies(talker rclcpp std_msgs)

#interface
add_executable(mav_interface src/mav_interface.cpp)
ament_target_dependencies(mav_interface rclcpp std_msgs std_srvs geometry_msgs)
target_include_directories(mav_interface PUBLIC include)
target_link_libraries(mav_interface MAVSDK::mavsdk)

target_sources(mav_interface PUBLIC src/quad.cpp)
#service
# add_executable(service_arm src/service_arm.cpp)
# ament_target_dependencies(service_arm rclcpp std_srvs)


# INSTALLATION
install(TARGETS
  # talker
  # service_arm
  mav_interface
  DESTINATION lib/${PROJECT_NAME})

# ament_export_dependencies(rosidl_default_runtime)
ament_package()