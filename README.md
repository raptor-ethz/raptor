# raptor
Raptor workspace for ROS 2 packages

# Workspace Structure
```
src/
|-  package_1/
    |-  include
    |-  src
    |-  srv
```

# ROS 2 cheatsheet

## Source setup files
Run on every new shell to have access to ros2 commands
```bash
source /opt/ros/humble/setup.bash
```




## Build, Install and Run

### Build
Build all packages, run the build command within the workspace's parent directory 'raptor'
```bash
colcon build
```
To build only a specific package, run
```bash
colcon build --packages-select <package_name>
```

### Install
To use executables, from inside the workspace's parent directory 'raptor'
```bash
. install/local_setup.bash
```

### Run
Run
```bash
ros2 run <package_name> <my_node>
```

## Create new package
Within raptor/src run
```bash
ros2 pkg create --build-type amend_cmake --dependencies rclcpp <package_name>
```
Note: '--dependencies rclcpp' will automatically include rclcpp as a dependency in package.xml and CMakeLists.txt.




## Create new node

### XML
Add dependencies in package.xml
```xml
<depend>rclcpp</depend>
```

### CMake
Add dependencies
```CMake
find_package(rclcpp REQUIRED)
```

Add target/executable
```CMake
add_executable(my_programm src/my_programm_src.cpp)
```

Link libraries to target
```CMake
ament_target_dependencies(my_programm ros_package) # link ros dependencies
target_link_libraries(my_programm external_package) # link standard dependencies
```

Add target to installation list
```CMake
install(TARGETS
  my_programm
  DESTINATION lib/${PROJECT_NAME})
```

### CPP
Include header files
```CPP
#include "rclcpp/rclcpp.hpp" // ros2 client library
#include "std_msgs/msg/string.hpp" // message definitions
```

## Create Interfaces

Message and service definitions must be placed in directories called msg and srv respectively.

IMPORTANT: Naming convention for messages and services: CamelCased

### Services
Define request and response variables in a MyService.srv file (placed in the srv directory) using the following structure:
```srv
int64 request_variable
---
int64 response_variable
```

Add the required dependency in the CMakeLists.txt.
```CMake
# required package for generating custom services
find_package(rosidl_default_generators REQUIRED)

# define which services should be generated
rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/MyService.srv"
  DEPENDENCIES <ros_msg> # If custom messages depend on ros_msgs
)
```

Add the required dependency in the package.xml.
```XML
<!-- build dependency -->
<build_depend>rosidl_default_generators</build_depend>

<!-- runtime dependency -->
<exec_depend>rosidl_default_runtime</exec_depend> 

<!-- name of dependency group to which the package belongs -->
<member_of_group>rosidl_interface_packages</member_of_group>
```

### Testing custom interface
Build and install package, then run
```bash
ros2 interface show my_package/msg/MyMessage
ros2 interface show my_package/srv/MyService
```