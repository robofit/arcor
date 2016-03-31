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
  FILES pickplace.action
)

generate_messages(
  DEPENDENCIES actionlib_msgs shape_msgs
)

catkin_package(
  CATKIN_DEPENDS 
    actionlib_msgs
    shape_msgs
    std_msgs
    geometry_msgs)
