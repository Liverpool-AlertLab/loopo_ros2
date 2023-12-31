cmake_minimum_required(VERSION 3.5)
project(loopo_interfaces)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/SendCommand.srv"
  "action/Homing.action"
  "action/Move.action"
  "action/Grasp.action"
  "action/Twist.action"
  DEPENDENCIES std_msgs
)

# Include Cpp "include" directory
include_directories(include)

ament_package()