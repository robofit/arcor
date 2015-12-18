# art_tablet_bridge

Communication bridge between artable PC and mobile device (tablet). It reads ROS messages with detected objects
and it sends them to the mobile device.

Nodes: 

* bridge_to_file.py
    * Subscribe topic with detected objects and store them to JSON file in local filesystem
* fake_sender.py
    * Sends fake objects position for testing purposes

## Dependecies: 
* jsonpickle
    * `$ sudo pip install jsonpickle`