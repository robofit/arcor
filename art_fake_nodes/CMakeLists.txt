cmake_minimum_required(VERSION 2.8.3)
project(art_fake_nodes)

find_package(catkin REQUIRED COMPONENTS
  actionlib
  geometry_msgs
  roscpp
  rospy
  sensor_msgs
  std_msgs
  std_srvs
  roslint
  roslaunch
)

set(ROSLINT_PYTHON_OPTS "--max-line-length=120")
roslint_python()
roslint_add_test()

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES art_fake_nodes
  CATKIN_DEPENDS geometry_msgs sensor_msgs std_msgs std_srvs
#  DEPENDS system_lib
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)
