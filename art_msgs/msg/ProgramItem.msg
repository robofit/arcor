uint16 id # id of the item (operation)
uint16 on_success # id of next item
uint16 on_failure # 0 means jump to block end

uint8 type
uint8 spec

# -----------------------------------------------------
# OPERATION TYPES
# -----------------------------------------------------

uint8 GET_READY=0 # retract arms etc.
uint8 MANIP_PICK=1
uint8 MANIP_PLACE=2
uint8 MANIP_PICK_PLACE=3
uint8 WAIT=4

# use pick_pose to pick object of given type
# place it to place_pose or place_polygon
uint8 MANIP_PICK_PLACE_FROM_FEEDER=5

# -----------------------------------------------------
# OPERATION PARAMETERS
# -----------------------------------------------------

string object # ID or type - based on operation type/specifier

geometry_msgs/PoseStamped pick_pose # for picking object with unknown ID -> exact pose
geometry_msgs/PolygonStamped pick_polygon # -> approximate location

geometry_msgs/PoseStamped place_pose # exact location
geometry_msgs/PolygonStamped place_polygon # approximate location

# -----------------------------------------------------
# OPERATION SPECIFIERS
# -----------------------------------------------------

# operation_spec: PICK, PLACE, PICK_PLACE
uint8 MANIP_ID=0 # do manipulation with concrete object
uint8 MANIP_TYPE=1 # do manipulation with any object of this type

# operation_spec: WAIT
uint8 WAIT_FOR_USER=0
uint8 WAIT_UNTIL_USER_FINISHES=1

# -----------------------------------------------------
# OTHER STUFF
# -----------------------------------------------------

# Text/graphic instructions for a human
SceneLabel[] labels
