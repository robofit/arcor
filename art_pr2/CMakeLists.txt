cmake_minimum_required(VERSION 2.8.3)
project(art_pr2)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  art_pr2_moveit_config
  roslaunch
  rgbd_launch
  openni_launch
#  pr2_bringup
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES art_pr2
#  CATKIN_DEPENDS roscpp
#  DEPENDS system_lib
)

# include_directories(include)
include_directories(
  ${catkin_INCLUDE_DIRS}
)

# TODO fix errors in pr2_bringup
#if (CATKIN_ENABLE_TESTING)
#  roslaunch_add_file_check(launch)
#endif()

install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch)

