cmake_minimum_required(VERSION 3.10)
project(light_detector)

## Use C++14
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

## By adding -Wall and -Werror, the compiler does not ignore warnings anymore,
## enforcing cleaner code.
add_definitions(-Wall -Werror)

## Add optimization flags
add_compile_options(-O2)

## Export compile commands for clangd
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

#######################
## Find dependencies ##
#######################

set(OpenGL_GL_PREFERENCE GLVND)


## CMAKE_PREFIX_PATH 
find_package(OpenGL REQUIRED)
find_package(ament_cmake_auto REQUIRED)
find_package(OpenCV REQUIRED)
find_package(fmt REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
ament_auto_find_build_dependencies()

if(NOT OpenCV_FOUND OR NOT fmt_FOUND)
  message(WARNING "OpenCV or fmt not found, skipping ${PROJECT_NAME}")
  return()
else ()
  message(STATUS "OpenCV and fmt found, building ${PROJECT_NAME}")
endif()



###########
## Build ##
###########

ament_auto_add_library(${PROJECT_NAME} SHARED
  src/detector_node.cpp
  src/detector.cpp
  src/pnp_solver.cpp
  src/kalman_filter.cpp
  include/detector_node.hpp
  include/detector.hpp
  include/pnp_solver.hpp
  include/kalman_filter.hpp
)

target_include_directories(${PROJECT_NAME} PUBLIC ${OpenCV_INCLUDE_DIRS} ${Sophus_INCLUDE_DIRS} /opt/ros/humble/include/image_transport/image_transport)

target_link_libraries(${PROJECT_NAME}
${OpenCV_LIBS}
  fmt::fmt
  tbb
)

rclcpp_components_register_node(${PROJECT_NAME}
  PLUGIN rm_auto_aim_dart::LightDetectorNode
  EXECUTABLE light_detector_node
)

ament_auto_package()
