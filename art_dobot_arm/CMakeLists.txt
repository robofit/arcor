cmake_minimum_required(VERSION 2.8.3)
project(art_dobot_arm)

find_package(catkin REQUIRED COMPONENTS
  art_msgs
  geometry_msgs
  rospy
  std_msgs
  roslint
  roslaunch
  tf
  dobot
)

set(ROSLINT_PYTHON_OPTS "--max-line-length=250")
roslint_python()
roslint_add_test()

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES art_dobot_arm
CATKIN_DEPENDS art_msgs geometry_msgs rospy std_msgs dobot
#  DEPENDS system_lib
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

if (CATKIN_ENABLE_TESTING)
  roslaunch_add_file_check(launch)
endif()

catkin_install_python(PROGRAMS src/dobot_client.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})