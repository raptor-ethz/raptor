cmake_minimum_required(VERSION 3.8)
project(vicon)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# ros dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
# custom packages
find_package(raptor_interface REQUIRED)


# targets
add_subdirectory(vicon_datastream)
add_executable(vicon src/vicon.cpp)
ament_target_dependencies(vicon 
  rclcpp
  raptor_interface)

target_include_directories(vicon PUBLIC include)
target_link_libraries(vicon ViconDataStreamSDK_lib)

# installation
install(TARGETS
  vicon
  DESTINATION lib/${PROJECT_NAME})

ament_package()