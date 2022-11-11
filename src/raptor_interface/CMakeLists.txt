cmake_minimum_required(VERSION 3.8)
project(raptor_interface)

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
find_package(rosidl_default_generators REQUIRED)
# external dependencies
find_package(MAVSDK REQUIRED)


# INTERFACES
rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/RequestArm.srv"
)


# TARGETS
add_executable(talker src/talker_node.cpp)
ament_target_dependencies(talker rclcpp std_msgs)
add_executable(mav_interface src/mav_interface.cpp)
ament_target_dependencies(mav_interface rclcpp std_msgs )
target_include_directories(mav_interface PUBLIC include)
target_link_libraries(mav_interface MAVSDK::mavsdk)
add_executable(service_arm src/service_arm.cpp)
ament_target_dependencies(service_arm rclcpp)


# INSTALLATION
install(TARGETS
  talker
  service_arm
  mav_interface
  DESTINATION lib/${PROJECT_NAME})


ament_package()