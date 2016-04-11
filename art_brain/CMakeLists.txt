cmake_minimum_required(VERSION 2.8.3)
project(art_brain)


find_package(catkin REQUIRED COMPONENTS
  art_msgs
  geometry_msgs
  roscpp
  rospy
  std_msgs
)

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES art_brain
#  CATKIN_DEPENDS art_msgs geometry_msgs roscpp rospy std_msgs
#  DEPENDS system_lib
)


## Specify additional locations of header files
## Your package locations should be listed before other locations
# include_directories(include)
include_directories(
  ${catkin_INCLUDE_DIRS}
)

## Declare a C++ library
# add_library(art_brain
#   src/${PROJECT_NAME}/art_brain.cpp
# )

## Add cmake target dependencies of the library
## as an example, code may need to be generated before libraries
## either from message generation or dynamic reconfigure
# add_dependencies(art_brain ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Declare a C++ executable
# add_executable(art_brain_node src/art_brain_node.cpp)

## Add cmake target dependencies of the executable
## same as for the library above
# add_dependencies(art_brain_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Specify libraries to link a library or executable target against
# target_link_libraries(art_brain_node
#   ${catkin_LIBRARIES}
# )
