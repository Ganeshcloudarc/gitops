cmake_minimum_required(VERSION 3.0.2)
project(autopilot_utils)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  autopilot_msgs
  costmap_2d
  geometry_msgs
  map_msgs
  nav_msgs
  roscpp
  rospy
  tf
  tf2
  tf2_geometry_msgs
  tf2_ros
  tf2_sensor_msgs
)

catkin_python_setup()

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS roscpp
#  DEPENDS other non-ROS libs
)
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)
add_library(${PROJECT_NAME}
  src/trajectory_common.cpp
  src/tf_utils.cpp
  src/pose_utils.cpp
  src/costmap_manager.cpp
  src/autonav_utils.cpp
  src/dubins.cpp
  src/dwa_path_ganerator.cpp
)
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES})
install(
  TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
)
install(
  DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)
