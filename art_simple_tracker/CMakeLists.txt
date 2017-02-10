cmake_minimum_required(VERSION 2.8.3)
project(art_simple_tracker)

find_package(catkin REQUIRED COMPONENTS
  art_msgs
  rospy
  roslint
  roslaunch
)

catkin_package(CATKIN_DEPENDS art_msgs)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

set(ROSLINT_PYTHON_OPTS "--max-line-length=250")
roslint_python()
roslint_add_test()

if (CATKIN_ENABLE_TESTING)
  roslaunch_add_file_check(launch)
endif()

install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch)

catkin_install_python(PROGRAMS src/tracker.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
