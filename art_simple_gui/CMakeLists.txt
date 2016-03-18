cmake_minimum_required(VERSION 2.8.3)
project(art_simple_gui)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  geometry_msgs
  image_geometry
  rospy
  std_msgs
)


catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES art_simple_gui
#  CATKIN_DEPENDS cv_bridge geometry_msgs image_geometry rospy std_msgs
#  DEPENDS system_lib
)

include_directories(include)
include_directories(
  ${catkin_INCLUDE_DIRS}
)


## Declare a C++ executable
# add_executable(art_simple_gui_node src/art_simple_gui_node.cpp)

## Add cmake target dependencies of the executable
## same as for the library above
# add_dependencies(art_simple_gui_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Specify libraries to link a library or executable target against
# target_link_libraries(art_simple_gui_node
#   ${catkin_LIBRARIES}
# )
