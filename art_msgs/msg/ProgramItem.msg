uint16 id # id of the item (operation)
uint16 on_success # id of next item
uint16 on_failure # 0 means jump to block end

uint16 type

# -----------------------------------------------------
# OPERATION TYPES
# -----------------------------------------------------

# basic instructions
uint16 GET_READY=0 # retract arms etc.

# synchronization with user
uint16 WAIT_FOR_USER=50
uint16 WAIT_UNTIL_USER_FINISHES=51

# manipulation - pick
uint16 PICK_FROM_POLYGON=100 # polygon, object (type)
uint16 PICK_FROM_FEEDER=101 # pose, object (type)
uint16 PICK_OBJECT_ID=102 # object (ID)

# manipulation - place
uint16 PLACE_TO_POSE=200 # pose, object (type)

# -----------------------------------------------------
# OPERATION PARAMETERS
# -----------------------------------------------------

string[] object
geometry_msgs/PoseStamped[] pose
geometry_msgs/PolygonStamped[] polygon

# -----------------------------------------------------
# OTHER STUFF
# -----------------------------------------------------

# Text/graphic instructions for a human
SceneLabel[] labels
