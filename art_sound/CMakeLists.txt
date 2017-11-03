cmake_minimum_required(VERSION 2.8.3)
project(art_sound)

find_package(catkin REQUIRED COMPONENTS
  art_msgs
  rospy
  std_srvs
)

catkin_python_setup()

catkin_package(
   CATKIN_DEPENDS art_msgs rospy art_utils
)