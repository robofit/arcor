cmake_minimum_required(VERSION 2.8.3)
project(art_pr2_grasping)

# set(CMAKE_BUILD_TYPE Debug)

find_package(catkin REQUIRED COMPONENTS 
  moveit_core 
  moveit_ros_planning 
  moveit_ros_planning_interface 
  moveit_visual_tools
  moveit_simple_grasps
  std_msgs
  geometry_msgs
  roscpp
  cmake_modules
  pr2_controllers_msgs
  actionlib
  actionlib_msgs
  shape_msgs
  roslint
  art_msgs
  pr2_moveit_config
  roslaunch
  rostest
  tf_conversions
)

find_package(Boost REQUIRED COMPONENTS thread)

catkin_package(
  CATKIN_DEPENDS 
    moveit_simple_grasps
    actionlib_msgs
    shape_msgs
    pr2_controllers_msgs
    art_msgs
    std_msgs
  DEPENDS Boost
)

include_directories(include ${catkin_INCLUDE_DIRS} ${Boost_INCLUDE_DIRS})

# Main executable
add_executable(pr2grasp src/node.cpp src/objects.cpp src/action_server.cpp src/pr2grasp.cpp)
add_dependencies(pr2grasp ${catkin_EXPORTED_TARGETS})
target_link_libraries(pr2grasp ${catkin_LIBRARIES} ${Boost_LIBRARIES})

set(ROSLINT_CPP_OPTS "--filter=-runtime/references,-runtime/int")
roslint_cpp()
roslint_add_test()

if (CATKIN_ENABLE_TESTING)
  # TODO fix roslaunch test - it cannot find pr2grasp node...
  # roslaunch_add_file_check(launch DEPENDENCIES ${art_pr2_grasping_EXPORTED_TARGETS})
#  add_rostest(tests/grasping.test)
endif()
