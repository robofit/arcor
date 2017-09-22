cmake_minimum_required(VERSION 2.8.3)
project(art_calibration)


find_package(catkin REQUIRED COMPONENTS
  art_msgs
  geometry_msgs
  roscpp
  rospy
  sensor_msgs
  std_msgs
  ar_track_alvar
  tf
  roslaunch
  roslint
  dynamic_reconfigure
)

catkin_python_setup()

generate_dynamic_reconfigure_options(
  cfg/Calibration.cfg

)

set(ROSLINT_PYTHON_OPTS "--max-line-length=250")
roslint_python()
roslint_add_test()

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES art_cluster_detector
  CATKIN_DEPENDS art_msgs geometry_msgs sensor_msgs std_msgs
#  DEPENDS system_lib
)



include_directories(
  ${catkin_INCLUDE_DIRS}
)

if (CATKIN_ENABLE_TESTING)
  roslaunch_add_file_check(launch)
endif()

install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch)

catkin_install_python(PROGRAMS src/calibration.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
