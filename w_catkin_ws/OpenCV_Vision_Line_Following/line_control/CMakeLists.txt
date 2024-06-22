cmake_minimum_required(VERSION 3.0.2)
project(line_control)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  geometry_msgs
  image_transport
  roscpp
  rospy
  sensor_msgs
  std_msgs
)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")

find_package(OpenCV REQUIRED)
 

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES line_control
  CATKIN_DEPENDS cv_bridge geometry_msgs image_transport roscpp rospy sensor_msgs std_msgs
 # DEPENDS system_lib
  DEPENDS OpenCV
)

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

link_directories(${OpenCV_LIB_DIR})

## Declare a C++ library
# add_library(${PROJECT_NAME}
#   src/${PROJECT_NAME}/opencv_line_detection.cpp
# )

## Declare a C++ executable
## With catkin_make all packages are built within a single CMake context
## The recommended prefix ensures that target names across packages don't collide
add_executable(line_control_node src/line_control_node.cpp)
target_link_libraries(line_control_node
   ${catkin_LIBRARIES}
   ${OpenCV_LIBRARIES}
)

