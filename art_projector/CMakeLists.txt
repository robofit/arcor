cmake_minimum_required(VERSION 2.8.3)
project(art_projector)

find_package(catkin REQUIRED COMPONENTS
  roslint
  roslaunch
  art_utils
  cv_bridge
  image_geometry
  message_filters
  rospy
  sensor_msgs
  std_msgs
  std_srvs
  tf
)

catkin_python_setup()

set(ROSLINT_PYTHON_OPTS "--max-line-length=120")
roslint_python()
roslint_add_test()

catkin_package(CATKIN_DEPENDS std_msgs sensor_msgs)

catkin_install_python(PROGRAMS src/projector_node.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

if (CATKIN_ENABLE_TESTING)
  roslaunch_add_file_check(launch)
endif()

install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch)
