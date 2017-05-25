# message type for synchronization of brain and interfaces
# interfaces/brain sends current state to /art/interface/state (brain's publisher is latched - for newly connected interfaces)

string BRAIN_ID=ART_BRAIN

# system states
# interfaces should wait until brain sends STATE_IDLE and then they may start learning/execution etc.
int16 STATE_UNKNOWN=-1
int16 STATE_INITIALIZING=0
int16 STATE_IDLE=1
int16 STATE_LEARNING=2
int16 STATE_PROGRAM_RUNNING=3
int16 STATE_PROGRAM_STOPPED=4
int16 STATE_PROGRAM_FINISHED=5

string interface_id # should be unique for each interface and BRAIN_ID for the brain
int16 system_state
time timestamp # when this state was updated (useful especially during learning - newer timestamp indicates that some interface changed program_current_item)

# valid for STATE_LEARNING, STATE_PROGRAM_*
uint16 program_id
uint16 block_id

# item currently being learned
ProgramItem program_current_item
KeyValue[] flags # flags associated to program_current_item (e.g. selected object id when doing type-based manipulation)

int32 error_severity

# program and learning error severities
int32 NONE = 0
int32 SEVERE = 1  # fatal, immediately shut down
int32 ERROR = 2  # cannot continue in current program
int32 WARNING = 3 # ask user what to do
int32 INFO = 4  # let user know about what happened, but continue in program/learning execution

int32 error_code

int32 ERROR_UNKNOWN = 0
int32 ERROR_ROBOT_HALTED = 1001

# program errors
int32 ERROR_OBJECT_MISSING = 1101
int32 ERROR_OBJECT_MISSING_IN_POLYGON = 1102
int32 ERROR_NO_GRIPPER_AVAILABLE = 1201
int32 ERROR_OBJECT_IN_GRIPPER = 1202
int32 ERROR_NO_OBJECT_IN_GRIPPER = 1203
int32 ERROR_PICK_FAILED = 1301
int32 ERROR_PICK_PLACE_SERVER_NOT_READY = 1302
int32 ERROR_PLACE_FAILED = 1401
