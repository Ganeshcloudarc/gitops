cmake_minimum_required(VERSION 3.0.2)
project(autopilot_utils)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS 
  autopilot_msgs
  vehicle_common
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
  jsk_recognition_msgs
  zed_interfaces
)

# Python setup (if you have Python scripts)
catkin_python_setup()

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS roscpp
)

include_directories(
  include
  ${CMAKE_CURRENT_SOURCE_DIR}  # Include the current source directory
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
  src/footprint_transform.cpp
)

target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES})

install(
  TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(
  DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)
