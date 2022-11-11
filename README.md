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



# Source, Build, Install, Run

## Source

Run on every new shell to have access to ros2 commands
```bash
source /opt/ros/humble/setup.bash
```

## Build

Build all packages, run the build command within the workspace's parent directory 'raptor'
```bash
colcon build
```
To build only a specific package, run
```bash
colcon build --packages-select <package_name>
```

## Install
For using executables, from inside the workspace's parent directory 'raptor'
```bash
. install/local_setup.bash
```

## Run
To launch a ros-node
```bash
ros2 run <package_name> <my_node>
```

# Create new Packages and Nodes

## Packages

Within raptor/src run
```bash
ros2 pkg create --build-type amend_cmake --dependencies rclcpp <package_name>
```
Note: '--dependencies rclcpp' will automatically include rclcpp as a dependency in package.xml and CMakeLists.txt.

## Nodes

### XML
Add dependencies in package.xml
```xml
<depend>rclcpp</depend>
```

### CMake
In CMakeLists.txt:
- add dependencies 
  ```CMake
  find_package(rclcpp REQUIRED)
  ```

- add target/executable
  ```CMake
  add_executable(my_programm src/my_programm_src.cpp)
  ```

- link libraries to target
  ```CMake
  ament_target_dependencies(my_programm rclcpp) # link ros dependencies
  target_link_libraries(my_programm external_package) # link standard dependencies
  ```

- add target to installation list
  ```CMake
  install(TARGETS
    my_programm
    DESTINATION lib/${PROJECT_NAME})
  ```

### CPP
Include header files
```CPP
#include "rclcpp/rclcpp.hpp" // ros2 client library
#include "std_msgs/msg/string.hpp" // message and service definitions
```

# Interfaces

## List and Call Interfaces from Commandline

- List all active services
  ```bash
  ros2 service list
  ```

- Show type of a service
  ```bash
  ros2 service type /my_service
  ```

- Show structure of a type
  ```bash
  ros2 interface show <TypeName>
  ```

- Call service from commandline
  ```bash
  ros2 service call /my_service <TypeName> "{request_variable_1: value, request_variable_2: value}"
  ```

## Custom Interfaces

- Available messages and services: [ros2 common interfaces](https://github.com/ros2/common_interfaces)
- Available types: [ros2 built in types](https://docs.ros.org/en/rolling/Concepts/About-ROS-Interfaces.html)

Message and service should be provided in separate interface package. \
Names **must** follow the *CamelCased* convention. \
Definitions must be placed in directories called msg and srv respectively. \

For services, define request and response variables in the **/srv/MyService.srv** file using the following structure:
```srv
type request_variable
---
type response_variable
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