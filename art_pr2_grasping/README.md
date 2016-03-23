Actionlib server for pick&place with PR2 robot. Can do pick&place, or pick and then (a bit later) place.

Tested only on ROS Hydro!

How to install dependencies:

````cd catkin_ws/src
git clone -b origin/hydro-devel https://github.com/dornhege/moveit_simple_grasps.git
git clone -b origin/hydro-devel https://github.com/dornhege/moveit_visual_tools.git
git clone -b tags/1.5.0 https://github.com/davetcoleman/rviz_visual_tools.git````

How to run it:
 - roslaunch pr2_moveit_config move_group.launch (in simulation)
 - roslaunch pr2_ft_moveit_config move_group.launch (with the real robot with FT sensor)
 - roslaunch art_pr2_grasping node.launch
 - rosrun art_pr2_grasping test_as.py (for testing purposes only)
 
Usage (pickplace.action description):
 - id: arbitrary name of the object
 - pose: pose for picking the object
 - pose2: pose where to place the object
 - bb: currently only BOX is supported and the largest dimension is chosen as the cube size
 - arm: which arm to use
 - operation: what to do
 
TODO:
 - more checks (e.g. check tactile sensors and detect object drop)
 - ability to use bounding box for grasp planning (not possible with moveit_simple_grasps released into ROS Hydro)
 - better configurability (e.g. read table size/pose from param)
