cmake_minimum_required(VERSION 2.8.3)
project(art_utils)

find_package(catkin REQUIRED COMPONENTS
  art_msgs
  geometry_msgs
  rospy
  roslint
)

catkin_python_setup()

set(ROSLINT_PYTHON_OPTS "--max-line-length=250")
roslint_python(src/art_utils/art_api_helper.py src/art_utils/interface_state_manager.py src/art_utils/program_helper.py)
roslint_add_test()

catkin_package(CATKIN_DEPENDS art_msgs)

if (CATKIN_ENABLE_TESTING)
  catkin_add_nosetests(test)
endif()

include_directories(
  ${catkin_INCLUDE_DIRS}
)
