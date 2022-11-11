# raptor
Raptor workspace for ROS 2 packages

# Workspace Structure
```
src/
|-  package_1/
    |-  src
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
ros2 pkg create --build-type amend_cmake <package_name>
```




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

Link libraries to target (ROS style)
```CMake
ament_target_dependencies(my_programm rclcpp std_msgs)
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