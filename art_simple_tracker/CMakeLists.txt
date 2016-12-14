cmake_minimum_required(VERSION 2.8.3)
project(art_simple_tracker)

find_package(catkin REQUIRED COMPONENTS
  art_msgs
  rospy
)

catkin_package()

include_directories(
  ${catkin_INCLUDE_DIRS}
)

if (CATKIN_ENABLE_TESTING)
  find_package(catkin REQUIRED COMPONENTS roslaunch)
  roslaunch_add_file_check(launch)
endif()
