cmake_minimum_required(VERSION 3.0.2)
project(autopilot_msgs)

find_package(catkin REQUIRED
    rospy
    message_generation
    std_msgs
    geometry_msgs
    geographic_msgs
    mavros_msgs
    )

## Generate messages in the 'msg' folder
add_message_files(
  DIRECTORY msg
  FILES
  FloatKeyValue.msg
  ControllerDiagnose.msg
  TrajectoryPoint.msg
  Trajectory.msg
 )

 ## Generate services in the 'srv' folder
add_service_files(
  FILES
  speedwaypoint.srv
 )
 
 generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
  geographic_msgs
  mavros_msgs
  sensor_msgs
)

catkin_package(
  CATKIN_DEPENDS roscpp  message_runtime 
)