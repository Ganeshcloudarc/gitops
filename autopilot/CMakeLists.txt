cmake_minimum_required(VERSION 3.0.2)
project(autopilot)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  ackermann_msgs
  nav_msgs
  dynamic_reconfigure
  geometry_msgs
  roscpp
  rospy
  sensor_msgs
  mavros
  mavros_extras
  geographic_msgs
  rosbridge_server
  )

generate_dynamic_reconfigure_options(
cfg/pid_config.cfg
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

