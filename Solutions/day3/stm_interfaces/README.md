# STM Interfaces

## Overview
`stm_interfaces` is a ROS 2 package that defines custom message and service types used for communication between ROS 2 nodes and STM microcontrollers. This package provides structured message and service definitions to facilitate control and data exchange.

## Package Structure
The package contains:
- **Custom message definitions** for publishing data and receiving control commands.
- **Custom service definitions** for requesting and setting control parameters.

### Directory Structure
```
stm_interfaces
├── include
├── msg
│   ├── STMControl.msg
│   ├── STMState.msg
├── srv
│   ├── STMGetControlVariables.srv
│   ├── STMSetControlType.srv
├── CMakeLists.txt
└── package.xml
```

## Installation and Usage
Ensure `stm_interfaces` is built and sourced within your ROS 2 workspace:
1. **Build the Package**:
   ```bash
   cd ~/ros2_ws
   colcon build --packages-select stm_interfaces
   ```
2. **Source the Workspace**:
   ```bash
   source ~/ros2_ws/install/setup.bash
   ```

## CMakeLists.txt Explained
### Key Sections
1. **Project Declaration and Standards**
```cmake
cmake_minimum_required(VERSION 3.5)
project(stm_interfaces)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()
```
**Explanation**:
- Specifies the minimum CMake version and project name.
- Sets the default C and C++ standards to C99 and C++14 if not already defined.

2. **Compiler Options**
```cmake
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()
```
**Explanation**:
- Adds extra compiler warnings for better code quality when using GCC or Clang.

3. **Finding Dependencies**
```cmake
# find dependencies
find_package(ament_cmake REQUIRED)
find_package(std_msgs REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(rosidl_default_generators REQUIRED)
```
**Explanation**:
- Finds the required packages for building and running the package, such as `ament_cmake` and `rosidl_default_generators` for generating ROS 2 interfaces.

4. **Message and Service File Declarations**
```cmake
set(msg_files
  "msg/STMState.msg"
  "msg/STMControl.msg"
)

set(srv_files
  "srv/STMGetControlVariables.srv"
  "srv/STMSetControlType.srv"
)
```
**Explanation**:
- Lists the `.msg` and `.srv` files that need to be compiled and generated.

5. **Generating Interfaces**
```cmake
rosidl_generate_interfaces(${PROJECT_NAME}
  ${msg_files}
  ${srv_files} 
  DEPENDENCIES builtin_interfaces std_msgs
)
```
**Explanation**:
- Generates ROS 2 interfaces from the specified message and service files.
- Specifies `builtin_interfaces` and `std_msgs` as dependencies for the generated code.

6. **Testing Setup**
```cmake
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()
```
**Explanation**:
- Configures the package for testing if `BUILD_TESTING` is enabled.
- Uses `ament_lint_auto` to automatically find and set up test dependencies.

7. **ament_package**
```cmake
ament_package()
```
**Explanation**:
- Marks the package as an `ament` package, enabling it to be built and used in ROS 2.

## Conclusion
The `stm_interfaces` package provides the necessary message and service types for effective communication with STM microcontrollers in a ROS 2 environment. By using these interfaces, nodes can publish and receive structured data for various control and monitoring purposes.

