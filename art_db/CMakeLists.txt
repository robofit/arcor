cmake_minimum_required(VERSION 2.8.3)
project(art_db)

find_package(catkin REQUIRED COMPONENTS
  art_msgs
  rospy
  mongodb_store
  roslint
)

set(ROSLINT_PYTHON_OPTS "--max-line-length=250")
roslint_python()
roslint_add_test()

catkin_package()

include_directories(
  ${catkin_INCLUDE_DIRS}
)

if (CATKIN_ENABLE_TESTING)
  find_package(catkin REQUIRED COMPONENTS roslaunch)
  roslaunch_add_file_check(launch)
endif()
