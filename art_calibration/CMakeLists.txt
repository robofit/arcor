cmake_minimum_required(VERSION 2.8.3)
project(art_calibration)


find_package(catkin REQUIRED COMPONENTS
  art_msgs
  geometry_msgs
  roscpp
  rospy
  sensor_msgs
  std_msgs
  ar_track_alvar
  tf
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES art_cluster_detector
#  CATKIN_DEPENDS art_msgs geometry_msgs roscpp rospy sensor_msgs std_msgs
#  DEPENDS system_lib
)

# catkin_python_setup()

include_directories(include)
include_directories(
  ${catkin_INCLUDE_DIRS}
)

## Declare a C++ executable
 add_executable(art_calibration_node src/node.cpp)

## Specify libraries to link a library or executable target against
target_link_libraries(art_calibration_node
  ${catkin_LIBRARIES}
)
