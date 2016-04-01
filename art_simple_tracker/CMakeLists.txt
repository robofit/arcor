cmake_minimum_required(VERSION 2.8.3)
project(art_simple_tracker)

find_package(catkin REQUIRED COMPONENTS
  art_object_recognizer_msgs
  rospy
)

catkin_package()

include_directories(
  ${catkin_INCLUDE_DIRS}
)
