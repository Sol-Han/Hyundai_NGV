cmake_minimum_required(VERSION 2.8.3)
project(turtle_control)

find_package(catkin REQUIRED COMPONENTS
  #dynamic_reconfigure
  message_generation
  roscpp
  #rosgraph_msgs
  rospy
  std_msgs
  geometry_msgs
  #cmake_modules
  #gps_common
)

#add_message_files(
#    FILES
#    GPSFix.msg
#)


generate_messages(
   DEPENDENCIES
   std_msgs
   geometry_msgs
)

catkin_package(
  CATKIN_DEPENDS message_runtime roscpp rospy std_msgs gps_common
)

include_directories(include ${catkin_INCLUDE_DIRS})


install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
)

install(PROGRAMS scripts/morin_usv_emul.py DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
