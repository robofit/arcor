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
  roslint
)

# catkin_python_setup()

set(ROSLINT_PYTHON_OPTS "--max-line-length=250")
roslint_python()
roslint_add_test()

catkin_package()

include_directories(
  ${catkin_INCLUDE_DIRS}
)
