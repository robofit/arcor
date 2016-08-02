# event / item of the whole interface state
int16 type

# system states
int16 STATE_UNKNOWN=-1
int16 STATE_INITIALIZING=0
int16 STATE_IDLE=1
int16 STATE_LEARNING=2 # idata=[prog_id, inst_id] # TODO instruction type??
int16 STATE_PROGRAM_RUNNING=3 # idata=[prog_id, inst_id]
int16 STATE_PROGRAM_STOPPED=4 # idata=[prog_id, inst_id]
int16 STATE_PROGRAM_FINISHED=5 # idata=[prog_id] TODO result?

# all regarding objects
int16 SELECTED_OBJECT_ID=100 # data
int16 SELECTED_OBJECT_TYPE=101 # data
int16 UNSELECTED_OBJECT_ID=102 # data, used only by interfaces
int16 UNSELECTED_OBJECT_TYPE=103 # data

# all regarding places
int16 SELECTED_PLACE=200 # place
int16 UNSELECTED_PLACE=201 # place

# all regarding polygons
int16 POLYGON= 301 # polygon -> published when whole polygon is set -> TODO publish points?

# interface events
int16 CLEAR_ALL=400 # clear all selected objects, places etc
int16 INT_ENABLED=401 # when some interface claims itself enabled others should set themselves disabled
int16 INT_DISABLED=402
int16 INT_READY=403 # when interface is fully initialized
int16 INT_NOT_READY=404

string[] data
int16[] idata

geometry_msgs/PolygonStamped polygon
geometry_msgs/PoseStamped place
geometry_msgs/PointStamped point
