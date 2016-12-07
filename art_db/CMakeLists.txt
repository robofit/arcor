cmake_minimum_required(VERSION 2.8.3)
project(art_db)

find_package(catkin REQUIRED COMPONENTS
  art_msgs
  rospy
  mongodb_store
)

catkin_package()

include_directories(
  ${catkin_INCLUDE_DIRS}
)
