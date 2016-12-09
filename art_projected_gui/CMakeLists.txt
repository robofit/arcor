cmake_minimum_required(VERSION 2.8.3)
project(art_projected_gui)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roslint
  art_msgs
  cv_bridge
  geometry_msgs
  image_geometry
  rospy
  std_msgs
)

catkin_python_setup()

roslint_python()
roslint_add_test()

catkin_package(
  CATKIN_DEPENDS geometry_msgs art_msgs std_msgs
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)
