uint8 id # id of the item (operation)
uint8 on_success # id of next item
uint8 on_failure

uint8 type
uint8 spec

string object # ID or type - based on operation type
geometry_msgs/PoseStamped pick_pose # for picking object with unknown ID -> exact pose
geometry_msgs/PolygonStamped pick_polygon # -> approximate location

geometry_msgs/PoseStamped place_pose # exact location
geometry_msgs/PolygonStamped place_polygon # approximate location

# operation types
uint8 GET_READY=0 # retract arms etc.
uint8 MANIP_PICK=1
uint8 MANIP_PLACE=2
uint8 MANIP_PICK_PLACE=3
uint8 WAIT=4

# operation_spec: PICK, PLACE, PICK_PLACE
uint8 MANIP_ID=0 # do manipulation with concrete object
uint8 MANIP_TYPE=1 # do manipulation with any object of this type

# operation_spec: WAIT
uint8 WAIT_FOR_USER=0
uint8 WAIT_UNTIL_USER_FINISHES=1

# Text/graphic instructions for a human
SceneLabel[] labels
