cmake_minimum_required(VERSION 2.8.3)
project(art_brain)


find_package(catkin REQUIRED COMPONENTS
  art_msgs
  geometry_msgs
  roscpp
  rospy
  std_msgs
  roslint
  roslaunch
)

catkin_python_setup()

set(ROSLINT_PYTHON_OPTS "--max-line-length=120")
roslint_python()
roslint_add_test()

catkin_package(CATKIN_DEPENDS art_msgs std_msgs geometry_msgs)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

if (CATKIN_ENABLE_TESTING)
  roslaunch_add_file_check(launch)
endif()

install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch)

catkin_install_python(PROGRAMS src/node.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})