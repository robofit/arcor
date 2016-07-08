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
)

# catkin_python_setup()


catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES art_basic_control
#  CATKIN_DEPENDS actionlib geometry_msgs roscpp rospy sensor_msgs std_msgs
#  DEPENDS system_lib
)

# include_directories(include)
include_directories(
  ${catkin_INCLUDE_DIRS}
)

## Declare a C++ executable
# add_executable(art_basic_control_node src/art_basic_control_node.cpp)

## Specify libraries to link a library or executable target against
# target_link_libraries(art_basic_control_node
#   ${catkin_LIBRARIES}
# )
