cmake_minimum_required(VERSION 3.8)
project(vicon_interface)

add_subdirectory(vicon_datastream)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# ros dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
# custom packages
find_package(raptor_interface REQUIRED)


# targets
add_executable(vicon_interface src/vicon_interface.cpp)
ament_target_dependencies(vicon_interface 
  rclcpp
  raptor_interface)

target_include_directories(vicon_interface PUBLIC include)
target_link_libraries(vicon_interface ViconDataStreamSDK_lib)

# installation
install(TARGETS
  vicon_interface
  DESTINATION lib/${PROJECT_NAME})

ament_package()