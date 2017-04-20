cmake_minimum_required(VERSION 2.8.3)
project(art_bridge)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  roscpp
  rospy
  std_msgs
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES art_bridge
#  CATKIN_DEPENDS geometry_msgs roscpp rospy std_msgs
#  DEPENDS system_lib
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

## Declare a C++ executable
# add_executable(art_bridge_node src/art_bridge_node.cpp)

## Add cmake target dependencies of the executable
## same as for the library above
# add_dependencies(art_bridge_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Specify libraries to link a library or executable target against
# target_link_libraries(art_bridge_node
#   ${catkin_LIBRARIES}
# )
