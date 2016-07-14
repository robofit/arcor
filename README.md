# ar-table-itable
Experimental stuff.

[![Build Status](https://travis-ci.org/robofit/ar-table-itable.svg)](https://travis-ci.org/robofit/ar-table-itable)

Related repositories:
 - https://github.com/robofit/ar-table-common
 - https://github.com/robofit/ar-table-pr2
 - https://github.com/robofit/ar-table-handheld
 
New launch - art_brain/launch/Table.launch - should run all nodes on table (pr2 has its own launch) - not stable, USE WITH CAUTION!

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
   * publishes events (e.g. which object was selected) to the `/art/interface/state` topic (`art_msgs/InterfaceState`) - there is a helper class for this in `art_interface_utils`
   * listens to events of other interfaces and updates according to them

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
