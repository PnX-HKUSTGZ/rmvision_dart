cmake_minimum_required(VERSION 3.8)
project(auto_aim_interfaces)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

## Add optimization flags
add_compile_options(-O2)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(asio_cmake_module REQUIRED)


rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/Light.msg"
  "msg/Lights.msg"
  "msg/DebugLight.msg"
  "msg/DebugLights.msg"
  "msg/Send.msg"
  

  DEPENDENCIES
    std_msgs
    geometry_msgs
    sensor_msgs
)


ament_package()