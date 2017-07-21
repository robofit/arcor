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
  FILES PickPlace.action LocalizeAgainstUMF.action LearningRequest.action
  ArmNavigation.action
)

add_message_files(
  FILES
  ObjInstance.msg
  ObjectType.msg
  InstancesArray.msg
  ObjectsCentroids.msg
  UserStatus.msg # deprecated
  UserActivity.msg # deprecated
  SystemState.msg
  SceneLabel.msg
  ProgramItem.msg
  ProgramBlock.msg
  Program.msg
  ProgramHeader.msg
  InterfaceState.msg
  KeyValue.msg
  Touch.msg
  User.msg
  UserArray.msg
)

add_service_files(
    FILES
    getProgram.srv
    getProgramHeaders.srv
    storeProgram.srv
    getObjectType.srv
    storeObjectType.srv
    startProgram.srv
    TouchCalibrationPoints.srv
    NotifyUser.srv
    ProgramErrorResolve.srv
    SelectArm.srv
    RecalibrateCell.srv
    ObjectFlagSet.srv
    ObjectFlagClear.srv
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
