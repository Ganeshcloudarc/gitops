cmake_minimum_required(VERSION 3.0.2)
project(vehicle_safety_cpp)
set(CMAKE_CXX_STANDARD 17)


find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  roslib
  
  autopilot_msgs
  message_generation

  # vehicle_stop_command
  # pilot
)
# add_message_files(
#   FILES
  
# )
add_service_files( 
   FILES 
   trigger.srv
)

generate_messages(
  DEPENDENCIES
  std_msgs
)


catkin_package(
  LIBRARIES ${PROJECT_NAME} 
  CATKIN_DEPENDS roscpp roslib std_msgs autopilot_msgs  message_runtime 
  INCLUDE_DIRS include
)

# catkin_package()
# add_dependencies(talker vehicle_safety_generate_messages_cpp)
# add_dependencies(talker ${catkin_EXPORTED_TARGETS})


#- Error :


# [ 92%] Linking CXX executable /home/vishnuk/catkin_ws/devel/lib/vehicle_safety_cpp/curve_detector
# /usr/bin/ld: CMakeFiles/curve_detector.dir/src/curve_detector.cpp.o: in function `main':
# curve_detector.cpp:(.text+0x12a): undefined reference to `RDP::DouglasPeucker::Simplify(std::vector<RDP::Point2d, std::allocator<RDP::Point2d> >&, float)'
# collect2: error: ld returned 1 exit status

#- solution 

add_library(DouglasPeucker src/DouglasPeucker.cpp)

add_executable(curve_detector_cpp src/curve_detector_cpp.cpp)
target_link_libraries(curve_detector_cpp ${catkin_LIBRARIES} DouglasPeucker)

add_executable(vehicle_safety_cpp src/vehicle_safety_cpp.cpp)
target_link_libraries(vehicle_safety_cpp ${catkin_LIBRARIES})
add_dependencies(vehicle_safety_cpp ${catkin_EXPORTED_TARGETS})

# add_dependencies(vehicle_safety_cpp_generate_messages_cpp autopilot_msgs) 

# add_dependencies(listener ${catkin_EXPORTED_TARGETS})
# add_dependencies(vehicle_safety_cpp ${catkin_EXPORTED_TARGETS})

include_directories(include ${catkin_INCLUDE_DIRS} {PROJECT_NAME})

add_executable(vehicle_safety_analyzer_cpp src/vehicle_safety_analyzer_cpp.cpp)
target_link_libraries(vehicle_safety_analyzer_cpp ${catkin_LIBRARIES})
add_dependencies(vehicle_safety_analyzer_cpp ${catkin_EXPORTED_TARGETS})

# target_link_libraries(curve_detector ${catkin_LIBRARIES})

# add_executable(DouglasPeucker src/DouglasPeucker.cpp) #these lines not needed as we are using douglas... as a library
# target_link_libraries(DouglasPeucker ${catkin_LIBRARIES})

#############
## Install ##
#############

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html

## Mark executable scripts (Python etc.) for installation
## in contrast to setup.py, you can choose the destination
# catkin_install_python(PROGRAMS
#   scripts/my_python_script
#   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark executables for installation
## See http://docs.ros.org/melodic/api/catkin/html/howto/format1/building_executables.html
# install(TARGETS ${PROJECT_NAME}_node
#   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark libraries for installation
## See http://docs.ros.org/melodic/api/catkin/html/howto/format1/building_libraries.html
# install(TARGETS ${PROJECT_NAME}
#   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
# )

## Mark cpp header files for installation
# install(DIRECTORY include/${PROJECT_NAME}/
#   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
#   FILES_MATCHING PATTERN "*.h"
#   PATTERN ".svn" EXCLUDE
# )

## Mark other files for installation (e.g. launch and bag files, etc.)
# install(FILES
#   # myfile1
#   # myfile2
#   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
# )

#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
# catkin_add_gtest(${PROJECT_NAME}-test test/test_vehicle_safely_cpp.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)