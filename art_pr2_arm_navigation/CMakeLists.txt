cmake_minimum_required(VERSION 2.8.3)
project(art_pr2_arm_navigation)

find_package(catkin REQUIRED COMPONENTS
  actionlib
  art_msgs
  geometry_msgs
  moveit_core
  moveit_ros_planning
  moveit_ros_planning_interface
  pr2_moveit_config
  rospy
  std_msgs
  roslint
)

catkin_python_setup()

set(ROSLINT_PYTHON_OPTS "--max-line-length=250")
roslint_python()
roslint_add_test()

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES art_pr2_arm_navigation
   CATKIN_DEPENDS art_msgs geometry_msgs std_msgs
#  DEPENDS system_lib
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

catkin_install_python(PROGRAMS src/node.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})