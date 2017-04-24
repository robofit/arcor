# indexes of body parts
int8 NOSE=0
int8 NECK=1
int8 RSHO=2
int8 RELB=3
int8 RWRI=4
int8 LSHO=5
int8 LELB=6
int8 LWRI=7
int8 RHIP=8
int8 RKNE=9
int8 RANK=10
int8 LHIP=11
int8 LKNE=12
int8 LANK=13
int8 LEYE=14
int8 REYE=15
int8 LEAR=16
int8 REAR=17

int8[] detected_parts # indexes of detected body parts
geometry_msgs/Point[18] parts

float32 score # range?

# ------------------------------------------
# everything below is filled by tracker

int8 user_id

int8 activity
int8 UNKNOWN=0
int8 READY=1 # in front of table, upright
int8 WORKING=2 # working on the tabletop
int8 AWAY=3 # tracked, but too far from table
