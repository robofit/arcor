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
)

catkin_python_setup()

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES art_pr2_arm_navigation
#  CATKIN_DEPENDS actionlib art_msgs geometry_msgs moveit_core moveit_ros_planning moveit_ros_planning_interface pr2_moveit_config rospy std_msgs
#  DEPENDS system_lib
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

