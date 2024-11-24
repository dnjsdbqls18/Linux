cmake_minimum_required(VERSION 3.0.2)
project(bld_400R4_motor_control)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs    
  sensor_msgs
)

catkin_package(
  INCLUDE_DIRS include 
  CATKIN_DEPENDS roscpp rospy std_msg sensor_msgs
  DEPENDS system_lib
)

###########
## Build ##
###########

include_directories(

  ${catkin_INCLUDE_DIRS}
)

add_executable(bld_400R4_motor_control_node src/bld_400R4_motor_control_node.cpp)
add_dependencies(bld_400R4_motor_control_node ${bld_400R4_motor_control_node_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(bld_400R4_motor_control_node ${catkin_LIBRARIES} )

