Actionlib server for pick&place with PR2 robot. Can do pick&place, or pick and then (a bit later) place.

Tested only on ROS Hydro!

How to install dependencies:

```
cd catkin_ws/src
wstool init .
wstool merge https://raw.githubusercontent.com/robofit/ar-table-pr2/master/art_pr2_grasping/artpr2grasping.rosinstall
wstool update
cd ..
rosdep install --from-paths src --ignore-src --rosdistro $ROS_DISTRO -y
```

How to run it:
 - roslaunch pr2_moveit_config move_group.launch (in simulation)
 - roslaunch pr2_ft_moveit_config move_group.launch (with the real robot with FT sensor)
 - roslaunch art_pr2_grasping node.launch
 - rosrun art_pr2_grasping test_as.py (for testing purposes only)
 
Usage (pickplace.action description):
 - id: arbitrary name of the object
 - pose: pose for picking the object
 - pose2: pose where to place the object
 - bb: currently only BOX is supported
 - arm: which arm to use
 - operation: what to do
 
TODO:
 - more checks (e.g. check tactile sensors and detect object drop)
 - better configurability (e.g. read table size/pose from param)
