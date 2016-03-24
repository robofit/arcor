cmake_minimum_required(VERSION 2.8.3)
project(art_fake_detector)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  art_object_recognizer_msgs
  geometry_msgs
  roscpp
  rospy
  sensor_msgs
  std_msgs
)

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)



catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES art_fake_detector
#  CATKIN_DEPENDS art_object_recognizer_msgs geometry_msgs roscpp rospy sensor_msgs std_msgs
#  DEPENDS system_lib
)



## Specify additional locations of header files
## Your package locations should be listed before other locations
# include_directories(include)
include_directories(
  ${catkin_INCLUDE_DIRS}
)

