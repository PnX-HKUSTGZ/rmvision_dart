cmake_minimum_required(VERSION 3.8)
project(rm_livox_fusion)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(auto_aim_interfaces REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_sensor_msgs REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(eigen3_cmake_module REQUIRED)
find_package(PCL REQUIRED)
find_package(Eigen3 REQUIRED)

add_definitions(${PCL_DEFINITIONS})

add_executable(cloud_accumulator_node
  src/rm_livox_fusion_node.cpp
)

ament_target_dependencies(cloud_accumulator_node
  rclcpp
  sensor_msgs
  std_msgs
  geometry_msgs
  tf2
  tf2_ros
  tf2_sensor_msgs
  pcl_conversions
)

target_include_directories(cloud_accumulator_node PRIVATE
  ${PROJECT_SOURCE_DIR}/include
  ${PCL_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
)

target_link_libraries(cloud_accumulator_node
  ${PCL_LIBRARIES}
)

install(TARGETS cloud_accumulator_node
  DESTINATION lib/${PROJECT_NAME}
)

add_executable(range_fusion_node
  src/range_fusion_node.cpp
)

ament_target_dependencies(range_fusion_node
  rclcpp
  sensor_msgs
  std_msgs
  geometry_msgs
  auto_aim_interfaces
  tf2
  tf2_ros
  tf2_sensor_msgs
  pcl_conversions
)

target_include_directories(range_fusion_node PRIVATE
  ${PROJECT_SOURCE_DIR}/include
  ${PCL_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
)

target_link_libraries(range_fusion_node
  ${PCL_LIBRARIES}
)

install(TARGETS range_fusion_node
  DESTINATION lib/${PROJECT_NAME}
)

add_executable(send_mux_node
  src/send_mux_node.cpp
)

ament_target_dependencies(send_mux_node
  rclcpp
  std_msgs
  auto_aim_interfaces
)

install(TARGETS send_mux_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include
)

ament_package()
