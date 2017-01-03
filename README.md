# ar-table-itable

[![Build Status](https://travis-ci.org/robofit/ar-table-itable.svg)](https://travis-ci.org/robofit/ar-table-itable)

ARTable - vision of a near future workspace, where human and robot may safely and effectively collaborate. Our main focus is on human-robot interaction and especially on robot programming - to make it feasible for any ordinary skilled worker. Currently, the system uses pointing gestures and touch sensitive table as an input and augmented reality as an feedback (projected user interface).

The video below shows how operator uses touch sensitive table and augmented reality interface to set parameters of a program consisting of two pick and place tasks. For each task, the operator sets object type to be picked up, area from which objects of that type should be picked and location on the table where the robot (PR2) should place them.

[![ARTable video](https://i.ytimg.com/vi/M_KxpIJo1LA/0.jpg)](https://youtu.be/M_KxpIJo1LA)

This is ARTable main repository. Related repositories:
 - https://github.com/robofit/ar-table-common
 - https://github.com/robofit/ar-table-pr2
 - https://github.com/robofit/ar-table-handheld
 

### System overview

TBD

### Contributing

 - Follow [PyStyleGuide](http://wiki.ros.org/PyStyleGuide) or [CppStyleGuide](http://wiki.ros.org/CppStyleGuide)
 - Use [catkin_lint](http://fkie.github.io/catkin_lint/) to check for common problems (```catkin_lint -W2 your_package_name```)
 - Use [roslint](http://wiki.ros.org/roslint) to run static analysis of your code.

### ARTable API

All topics, parameters and services can be found in `/art` namespace.

#### General, common stuff

Parameter with name of the world frame - TBD.

#### Database

Persistent storage for ARTable (sqlite).

Services:
```
  /art/db/object/get
    Node: art_db/db.py
    Type: art_msgs/getObject
    Args: obj_id
  /art/db/object/store
    Node: art_db/db.py
    Type: art_msgs/storeObject
    Args: obj_id name model_url type bbox
  /art/program/get
    Node: art_db/db.py
    Type: art_msgs/getProgram
    Args: id
  /art/program/store
    Node: art_db/db.py
    Type: art_msgs/storeProgram
    Args: program
```

#### User

Topics:
````
  /art/user/status
    Description: ID of the user + tracking information
    Node: ?? skeleton tracking ??
    Type: art_msgs/UserStatus
  /art/user/activity
    Description: Simple activity recognition of the currently tracked user
    Node: art_table_pointing/node.launch
    Type: art_msgs/UserActivity
  /art/user/pointing_left
    Node: art_table_pointing/node.launch
    Type: geometry_msgs/PoseStamped
  /art/user/pointing_right
    Node: art_table_pointing/node.launch
    Type: geometry_msgs/PoseStamped
````

#### Interfaces in general

All interfaces live in `/art/interface` namespace. Each interface:
   * has `enable` and `disable` (empty) services
    * when one interface gets enabled others should disable themselves
    * disabled interface should still show everything, just ignore user inputs
   * publishes events (e.g. which object was selected) to the `/art/interface/events` topic (`art_msgs/InterfaceState`)
   * listens to events of other interfaces and updates according to them
   * listens to the latched topic `/art/interface/state` (`art_msgs/InterfaceState`) published by art_brain - this topic is especially usefull at interface startup / restart etc.
  
There is a rospy helper class for publishing and listening to events in `art_interface_utils` - the class has methods as `select_object_id(obj_id)` and callback for state updates.

##### Projected GUI (`art_projected_gui`)

2D user interface - projected on the table.

Topics:
````
  /art/interface/projected_gui/calibrated
    Type: std_msgs/Bool
````    
Services:
````
  /art/interface/projected_gui/show_marker
    Type: std_srvs/Empty
  /art/interface/projected_gui/hide_marker
    Type: std_srvs/Empty
  /art/interface/projected_gui/calibrate
    Type: std_srvs/Empty
````
Parameters:
````
  /art/interface/projected_gui/calibration_matrix
````

#### Objects

API related to object detection and tracking.

Topics:
````
  /art/object_detector/object
    Type: art_msgs/InstancesArray
  /art/object_detector/object_filtered
    Type: art_msgs/InstancesArray
````
