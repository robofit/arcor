cmake_minimum_required(VERSION 2.8.3)
project(art_collision_env)

find_package(catkin REQUIRED COMPONENTS
  rospy
  std_srvs
)

catkin_python_setup()

catkin_package(
  CATKIN_DEPENDS rospy std_srvs
)