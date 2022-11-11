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

# INTERFACES
rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/request_arm.srv"
  DEPENDENCIES geometry_msgs # Add packages that above messages depend on, in this case geometry_msgs for Sphere.msg
)

#THIRD-PARTY DEPENDENCIES
find_package(MAVSDK REQUIRED)

# TARGETS
add_executable(talker src/talker_node.cpp)
ament_target_dependencies(talker rclcpp std_msgs) # link libs to target in ros2 style I suppose?
add_executable(mav_interface src/mav_interface.cpp)
ament_target_dependencies(mav_interface rclcpp std_msgs ) # link libs to target in ros2 style I suppose?

#LINK THIRD-PARTY TARGETS
target_link_libraries(mav_interface MAVSDK::mavsdk)

target_include_directories(mav_interface PUBLIC include)

add_executable(service_arm src/service_arm.cpp)
ament_target_dependencies(service_arm rclcpp)

# INSTALLATION
install(TARGETS
  talker
  service_arm
  mav_interface
  DESTINATION lib/${PROJECT_NAME})


# probably unnessecary
# if(BUILD_TESTING)
#   find_package(ament_lint_auto REQUIRED)
#   # the following line skips the linter which checks for copyrights
#   # comment the line when a copyright and license is added to all source files
#   set(ament_cmake_copyright_FOUND TRUE)
#   # the following line skips cpplint (only works in a git repo)
#   # comment the line when this package is in a git repo and when
#   # a copyright and license is added to all source files
#   set(ament_cmake_cpplint_FOUND TRUE)
#   ament_lint_auto_find_test_dependencies()
# endif()

ament_package()
