cmake_minimum_required(VERSION 3.0.2)
project(autopilot_boson)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  ackermann_msgs
  dynamic_reconfigure
  geometry_msgs
  roscpp
  rospy
  sensor_msgs
  )

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES autopilot_boson
#  CATKIN_DEPENDS ackermann_msgs dynamic_reconfigure geometry_msgs roscpp rospy sensor_msgs
#  DEPENDS system_lib
)

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)


