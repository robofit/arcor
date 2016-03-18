cmake_minimum_required(VERSION 2.8.3)
project(art_simple_point_finger_detector)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  pcl_ros
  pcl_conversions
  roscpp
  rospy
  sensor_msgs
  std_msgs
  cv_bridge
)

find_package(
  PCL 1.2 REQUIRED
)
find_package(OpenCV REQUIRED)

add_definitions(
  ${PCL_DEFINITIONS}
)

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)



catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES art_simple_point_finger_detector
#  CATKIN_DEPENDS geometry_msgs pcl_conversion roscpp rospy sensor_msgs std_msgs
#  DEPENDS system_lib
)

include_directories(${OpenCV_INCLUDE_DIRS})
 include_directories(include)
include_directories(
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_LIBRARIES}
)



## Declare a C++ executable
add_executable(art_simple_point_finger_detector_node src/node.cpp)

## Specify libraries to link a library or executable target against
target_link_libraries(art_simple_point_finger_detector_node
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
 )
