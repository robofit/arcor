cmake_minimum_required(VERSION 2.8.3)
project(art_msgs)

find_package(catkin REQUIRED COMPONENTS
  actionlib
  actionlib_msgs
  std_msgs
  geometry_msgs
  shape_msgs
  message_generation)

add_action_files(
  DIRECTORY action
  FILES pickplace.action LocalizeAgainstUMF.action
)

add_message_files(
  FILES
  ObjInstance.msg
  InstancesArray.msg
  ObjectsCentroids.msg
  UserStatus.msg
  UserActivity.msg
  SystemState.msg
  ProgramItem.msg
  Program.msg
  ProgramArray.msg
  InterfaceState.msg
  InterfaceStateItem.msg
)

add_service_files(
    FILES
    getProgram.srv
    storeProgram.srv
    getObject.srv
    storeObject.srv
    startProgram.srv
)

generate_messages(
  DEPENDENCIES actionlib_msgs shape_msgs
)

catkin_package(
  CATKIN_DEPENDS 
    actionlib_msgs
    shape_msgs
    std_msgs
    geometry_msgs
    message_runtime
    )
