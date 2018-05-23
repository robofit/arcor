The node manages detected and artificial objects within the system. All topics and services can be found in ```/art/collision_env/``` namespace.

### Detected objects

The node listens to topic with tracked objects and publishes them to the planning scene.

#### Services

  * ```pause``` (std_srvs/Empty) - ignore changes in detected objects, do not publish to the planning scene.
  * ```resume``` (std_srvs/Empty) - resume normal operation.
  * ```detected/clear/all``` (std_srvs/Empty) - clear all objects. During normal operation, objects will be added again soon.
  * ```detected/clear/on_table``` (std_srvs/Empty) - clear all detected object on the table.
  * ```detected/clear/except``` ([art_msgs/StringTrigger](https://github.com/robofit/artable-msgs/blob/master/art_msgs/srv/StringTrigger.srv)) - clear detected objects with one exception (object_id).
  * ```detected/out_of_table/clear/except``` ([art_msgs/StringTrigger](https://github.com/robofit/artable-msgs/blob/master/art_msgs/srv/StringTrigger.srv)) - clear objects which are not on the table with one exception.
  * ```detected/set_pose``` ([art_msgs/PoseStampedTrigger](https://github.com/robofit/artable-msgs/blob/master/art_msgs/srv/PoseStampedTrigger.srv)) - set pose for one object (object_id). Usefull especially in paused state.
  
#### Topics

  * ```paused``` (std_msgs/Bool) - true when publishing to planning scene is paused.
  
### Artificial objects

So far, the artificial collision objects are defined as [art_msgs/CollisionPrimitive](https://github.com/robofit/artable-msgs/blob/master/art_msgs/msg/CollisionPrimitive.msg). On startup, the node loads them from [database](https://github.com/robofit/artable/tree/master/art_db), which could be initialized by a [setup init script](https://github.com/robofit/artable-setup-1/blob/master/art_setup_1/scripts/init.py).

#### Services

  * ```artificial/reload``` (std_srvs/Empty) - clear all artificial objects and load them from database.
  * ```artificial/clear/all``` (std_srvs/Empty) - clear all artificial objects (those are not cleared from database!).
  * ```artificial/clear/name``` ([art_msgs/StringTrigger](https://github.com/robofit/artable-msgs/blob/master/art_msgs/srv/StringTrigger.srv) - clear specific object.
  * ```artificial/save``` ([art_msgs/StringTrigger](https://github.com/robofit/artable-msgs/blob/master/art_msgs/srv/StringTrigger.srv) - save specific object to the permanent storage.
  * ```artificial/save_all``` (std_srvs/Empty) - save all objects to the permanent storage.
  * ```artificial/add/primitive``` ([art_msgs/AddCollisionPrimitive](https://github.com/robofit/artable-msgs/blob/master/art_msgs/srv/AddCollisionPrimitive.srv) - add new collision primitive or update the existing one. If name is not provided, it will be generated.

#### Topics

 * ```artificial``` ([art_msgs/CollisionObjects](https://github.com/robofit/artable-msgs/blob/master/art_msgs/msg/CollisionObjects.msg)) - list of artificial objects. The topic is latched and published on change (objects added, moved etc).
 
 ### Interactive markers
 
 Most of the operations could be done using interactive markers (```markers/update```). Each object has context menu and there is a box (above 0,0 point) with context menu for global operations.
 
 [![interactive markers](https://i.imgur.com/qRcWEnZt.png)](https://imgur.com/qRcWEnZ) [![interactive markers](https://i.imgur.com/GtMjpdYt.png)](https://imgur.com/GtMjpdY) [![interactive markers](https://i.imgur.com/eNx6IH8t.png)](https://imgur.com/eNx6IH8)
 
 ### TODO
 
   * Tests.
   * Interactive markers - scaling of artificial objects.
   * Artificial object grouping - to move more objects at once.
   * Locking - to allow / manage multi-user interaction.
