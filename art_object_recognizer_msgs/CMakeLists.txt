cmake_minimum_required(VERSION 2.8.3)
project(object_recognizer_msgs)

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
  std_msgs  # Or other packages containing msgs
  geometry_msgs
)


catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES object_recognizer_msgs
  CATKIN_DEPENDS roscpp message_runtime
#  DEPENDS system_lib
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)
