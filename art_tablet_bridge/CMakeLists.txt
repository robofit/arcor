cmake_minimum_required(VERSION 2.8.3)
project(art_tablet_bridge)


find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
  art_object_recognizer_msgs
)


catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES art_tablet_bridge
#  CATKIN_DEPENDS rospy std_msgs
#  DEPENDS system_lib
)

include_directories(include)
include_directories(
  ${catkin_INCLUDE_DIRS}
)
