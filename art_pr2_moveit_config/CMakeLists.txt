cmake_minimum_required(VERSION 2.8.3)
project(art_pr2_moveit_config)

find_package(catkin REQUIRED COMPONENTS roslaunch)

catkin_package()

if (CATKIN_ENABLE_TESTING)
  roslaunch_add_file_check(launch)
endif()

install(DIRECTORY launch DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  PATTERN "setup_assistant.launch" EXCLUDE)
install(DIRECTORY config DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
