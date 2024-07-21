cmake_minimum_required(VERSION 3.0.2)
project(m_car_km_serial_control)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  cv_bridge
  geometry_msgs
  image_transport
  sensor_msgs
)
find_package(OpenCV REQUIRED)


catkin_package(
  #INCLUDE_DIRS include
  #LIBRARIES m_car
  CATKIN_DEPENDS roscpp rospy std_msg geometry_msgs
  DEPENDS system_lib
)

###########
## Build ##
###########

include_directories(

  ${catkin_INCLUDE_DIRS}
)

add_executable(m_car_serial_control_km_node src/m_car_serial_control_km_node.cpp)

add_dependencies(m_car_serial_control_km_node ${m_car_serial_control_km_node_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

target_link_libraries(m_car_serial_control_km_node ${catkin_LIBRARIES} )


