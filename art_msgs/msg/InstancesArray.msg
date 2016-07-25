std_msgs/Header header
art_msgs/ObjInstance[] instances

# following two entries are meant to help nodes to react on new/lost objects
string[] new_objects # newly detected objects
string[] lost_objects # disappeared objects
