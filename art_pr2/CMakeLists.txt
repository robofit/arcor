cmake_minimum_required(VERSION 2.8.3)
project(art_pr2)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  pr2_moveit_config
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

