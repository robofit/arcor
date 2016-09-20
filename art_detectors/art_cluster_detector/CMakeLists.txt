cmake_minimum_required(VERSION 2.8.3)
project(art_cluster_detector)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  art_msgs
  geometry_msgs
  roscpp
  rospy
  sensor_msgs
  std_msgs
  pcl_ros
  pcl_conversions
)

find_package(
  PCL 1.2 REQUIRED
)


add_definitions(
  ${PCL_DEFINITIONS}
)


catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES art_cluster_detector
#  CATKIN_DEPENDS art_msgs geometry_msgs roscpp rospy sensor_msgs std_msgs
#  DEPENDS system_lib
)

###########
## Build ##
###########


include_directories(include)
include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(art_cluster_detector_node src/node.cpp)
add_dependencies(art_cluster_detector_node art_msgs_generate_messages_cpp)

target_link_libraries(art_cluster_detector_node
   ${catkin_LIBRARIES}
   ${PCL_LIBRARIES}
)

