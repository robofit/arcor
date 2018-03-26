cmake_minimum_required(VERSION 2.8.3)
project(art_arcode_detector)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  ar_track_alvar
  art_msgs
  rospy
  std_msgs
  roslaunch
  roslint
)

set(ROSLINT_PYTHON_OPTS "--max-line-length=120")
roslint_python()
roslint_add_test()

catkin_package(
  CATKIN_DEPENDS art_msgs std_msgs
)

if (CATKIN_ENABLE_TESTING)
  roslaunch_add_file_check(launch)
endif()

install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch)

catkin_install_python(PROGRAMS src/detector.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
