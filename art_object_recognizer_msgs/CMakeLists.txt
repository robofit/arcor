cmake_minimum_required(VERSION 2.8.3)
project(art_object_recognizer_msgs)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  geometry_msgs
  message_generation
)


add_message_files(
  FILES
  ObjInstance.msg
  InstancesArray.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs 
  geometry_msgs
)


catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES art_object_recognizer_msgs
  CATKIN_DEPENDS roscpp message_runtime
#  DEPENDS system_lib
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)
