cmake_minimum_required(VERSION 3.8)
project(raptor_interface)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)

set(srv_files
  "srv/GoToPos.srv"
)
rosidl_generate_interfaces(${PROJECT_NAME}
  ${srv_files}
)


ament_package()