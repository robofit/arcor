cmake_minimum_required(VERSION 2.8.3)
project(art_projected_gui)

find_package(catkin REQUIRED COMPONENTS
  roslint
  art_msgs
  cv_bridge
  geometry_msgs
  image_geometry
  rospy
  std_msgs
  roslaunch
  rostest
  art_utils
)

catkin_python_setup()

set(ROSLINT_PYTHON_OPTS "--max-line-length=250")
roslint_python()
roslint_add_test()

catkin_package(
  CATKIN_DEPENDS geometry_msgs art_msgs std_msgs
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

if (CATKIN_ENABLE_TESTING)
  roslaunch_add_file_check(launch)
  # catkin_add_nosetests(tests/test_ui_core.py)
  add_rostest(tests/ui_core.test)
  add_rostest(tests/ui_core_ros.test)
endif()

install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch)
