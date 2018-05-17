cmake_minimum_required(VERSION 2.8.3)
project(art_led)

find_package(catkin REQUIRED COMPONENTS
  roslint
  art_msgs
  rospy
  std_msgs
  std_srvs
)

catkin_python_setup()

set(ROSLINT_PYTHON_OPTS "--max-line-length=120")
roslint_python()
roslint_add_test()

catkin_package(
  CATKIN_DEPENDS art_msgs rospy std_msgs std_srvs

)
