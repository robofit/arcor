cmake_minimum_required(VERSION 2.8.3)
project(art_table_pointing)

set(CMAKE_CXX_FLAGS "-std=c++0x ${CMAKE_CXX_FLAGS} -g")

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  tf
)

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)

set(CMAKE_BUILD_TYPE Debug)

catkin_package(
  INCLUDE_DIRS include
#  LIBRARIES but_pr2_pge
#  CATKIN_DEPENDS roscpp tf
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(include)
include_directories(
  ${catkin_INCLUDE_DIRS}
)


## Declare a cpp executable
add_executable(art_table_pointing_kinect src/kinect_node.cpp)


## Specify libraries to link a library or executable target against
target_link_libraries(art_table_pointing_kinect
  ${catkin_LIBRARIES}
)
