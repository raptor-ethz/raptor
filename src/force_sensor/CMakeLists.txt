cmake_minimum_required(VERSION 3.13)

project(force_sensor)

# Fix issue with .c file in libATIDAQ
add_compile_options(-fcommon)

add_subdirectory(U3)
add_subdirectory(libATIDAQ)

# DEPENDENCIES
# ros default
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_srvs REQUIRED)
# ros custom


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
# readLabjack
add_executable(readLabjack src/readLabJack.cpp)
ament_target_dependencies(readLabjack
  rclcpp 
  std_msgs 
  std_srvs 
  geometry_msgs
)
target_include_directories(readLabjack PUBLIC
"$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/U3>"
"$<INSTALL_INTERFACE:include/${PROJECT_NAME}>")
target_link_libraries(readLabjack
  U3
  labjackusb
)

# ftConverter
add_executable(ftConverter src/ftconverter.cpp)
ament_target_dependencies(ftConverter
  rclcpp 
  std_msgs 
  std_srvs 
  geometry_msgs
)
target_include_directories(ftConverter PUBLIC
"$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/libATIDAQ>"
"$<INSTALL_INTERFACE:include/${PROJECT_NAME}>")
target_link_libraries(ftConverter
  ATIDAQ
)

# INSTALLATION
install(TARGETS
  ftConverter
  readLabjack
  DESTINATION lib/${PROJECT_NAME})

ament_package()