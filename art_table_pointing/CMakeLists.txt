cmake_minimum_required(VERSION 2.8.3)
project(art_table_pointing)

set(CMAKE_CXX_FLAGS "-std=c++0x ${CMAKE_CXX_FLAGS} -g")


find_package(catkin REQUIRED COMPONENTS
  roscpp
  tf
  art_msgs
  roslaunch
)

# set(CMAKE_BUILD_TYPE Debug)

catkin_package(
  CATKIN_DEPENDS roscpp tf art_msgs
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(art_table_pointing_kinect src/kinect_node.cpp)
add_dependencies(art_table_pointing_kinect art_msgs_generate_messages_cpp)

target_link_libraries(art_table_pointing_kinect
  ${catkin_LIBRARIES}
)

if (CATKIN_ENABLE_TESTING)
  roslaunch_add_file_check(launch)
endif()
