cmake_minimum_required(VERSION 2.8.3)
project(art_touch_driver)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  roscpp
  rospy
  std_msgs
  std_srvs
  roslint
)

set(ROSLINT_PYTHON_OPTS "--max-line-length=250")
roslint_python()
roslint_add_test()

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES art_touch_driver
  CATKIN_DEPENDS geometry_msgs rospy std_msgs std_srvs
#  DEPENDS system_lib
)

# include_directories(include)
include_directories(
  ${catkin_INCLUDE_DIRS}
)

## Declare a C++ executable
# add_executable(art_touch_driver_node src/art_touch_driver_node.cpp)


## Specify libraries to link a library or executable target against
# target_link_libraries(art_touch_driver_node
#   ${catkin_LIBRARIES}
# )

