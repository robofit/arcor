cmake_minimum_required(VERSION 2.8.3)
project(art_db)

find_package(catkin REQUIRED COMPONENTS
  art_msgs
  rospy
  mongodb_store
  roslint
  art_utils
  roslaunch
  rostest
)

set(ROSLINT_PYTHON_OPTS "--max-line-length=120")
roslint_python()
roslint_add_test()

catkin_package(CATKIN_DEPENDS art_msgs art_utils)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

if (CATKIN_ENABLE_TESTING)
  roslaunch_add_file_check(launch)
  add_rostest(tests/art_db.test DEPENDENCIES ${art_msgs_EXPORTED_TARGETS})
endif()

install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch)

catkin_install_python(PROGRAMS src/db.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
