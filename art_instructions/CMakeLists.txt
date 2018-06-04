cmake_minimum_required(VERSION 2.8.3)
project(art_instructions)

find_package(catkin REQUIRED COMPONENTS
  art_utils
  rospy
  roslint
)

catkin_python_setup()

set(ROSLINT_PYTHON_OPTS "--max-line-length=120")
roslint_python()
roslint_add_test()

catkin_package()