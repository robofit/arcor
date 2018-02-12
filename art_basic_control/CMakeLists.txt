cmake_minimum_required(VERSION 2.8.3)
project(art_basic_control)


find_package(catkin REQUIRED COMPONENTS
  actionlib
  geometry_msgs
  roscpp
  rospy
  sensor_msgs
  std_msgs
  pr2_controllers_msgs
  roslint
  roslaunch
  pr2_mechanism_msgs
  pr2_mannequin_mode
  pr2_controller_manager
)

# catkin_python_setup()

set(ROSLINT_PYTHON_OPTS "--max-line-length=250")
roslint_python()
roslint_add_test()

catkin_package(CATKIN_DEPENDS sensor_msgs std_msgs geometry_msgs pr2_controllers_msgs pr2_mechanism_msgs)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

if (CATKIN_ENABLE_TESTING)
  roslaunch_add_file_check(launch)
endif()

catkin_install_python(PROGRAMS src/node.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
