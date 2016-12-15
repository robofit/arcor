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
time timestamp # when this state was updated (usefull especially during learning - newer timestamp indicates that some interface changed program_current_item)

# valid for STATE_LEARNING, STATE_PROGRAM_*
uint16 program_id
uint16 block_id

# item currently being learned
ProgramItem program_current_item
KeyValue[] flags # flags associated to program_current_item (e.g. selected object id when doing type-based manipulation)
