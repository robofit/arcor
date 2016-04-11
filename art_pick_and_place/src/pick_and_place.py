#!/usr/bin/env python

import rospy
import rospkg
import sys
import actionlib
import random
import math

from art_msgs.msg import InstancesArray, ObjInstance
from art_msgs.msg import pickplaceAction, pickplaceGoal
from std_msgs.msg import String
from geometry_msgs.msg import PoseStamped

objects = {}
# STATES = ['nothing', 'picked']
NOTHING = 0
PICKED = 1
object_key = None
timestamp = None

class PickAndPlace:
    def __init__(self):
        self.client = actionlib.SimpleActionClient('/pr2_pick_place/pp', pickplaceAction)
        self.client.wait_for_server()
        self.goal_id = 0

    def pick(self, obj):
        goal = pickplaceGoal()

        self.goal_id += 1
        goal.id = "asdf"

        goal.operation = goal.PICK

        goal.keep_orientation = False
        print "asdf \n\n"
        print obj
        goal.bb = obj['bbox']

        goal.arm = goal.LEFT_ARM

        goal.pose = PoseStamped()
        goal.pose.header.frame_id = obj['frame_id']
        goal.pose.header.stamp = rospy.Time.now()
        goal.pose.pose = obj['pose']
        goal.pose.pose.position.z -= obj['bbox'].dimensions[2]/2

        rospy.loginfo('sending goal')
        self.client.send_goal(goal)

        self.client.wait_for_result()
        pass

    def place(self, pose, obj):
        """

        :type pose: PoseStamped
        :return:
        """
        goal = pickplaceGoal()
        goal.operation = goal.PLACE
        goal.z_axis_angle_increment = (2*3.14)/360*90
        self.goal_id += 1
        goal.id = self.goal_id
        goal.pose2 = PoseStamped()

        goal.pose2 = pose
        goal.pose2.header.stamp = rospy.Time.now()
        goal.pose2.pose.position.z = 0.74 + 0.06# + obj.bbox.dimensions[2]/2


def obj_cb(objects_data):
    global objects
    """

        :type objects_data: InstancesArray
        :return:
    """
    objects = {}
    for obj in objects_data.instances:
        if isinstance(obj, ObjInstance):
            objects[obj.object_id] = {'pose': obj.pose, 'bbox': obj.bbox, 'frame_id': objects_data.header.frame_id}
    

def point_cb(point_data):
    global objects
    global object_key
    global timestamp
    #print len(objects)
    min = [None, 1]
    
    for key,obj in objects.iteritems():
        
        diff = math.sqrt(math.pow((obj['pose'].position.x - point_data.pose.position.x),2) + 
                    math.pow((obj['pose'].position.x - point_data.pose.position.x),2) + 
                    math.pow((obj['pose'].position.x - point_data.pose.position.x),2)) 
	
        if diff < min[1]:
            min[1] = diff
            min[0] = key
    print min[1]    
    if min[1] < 0.1:
        print min[0]
        if timestamp is None:
            timestamp = rospy.Time.now()
        elif (rospy.Time.now() - timestamp) > rospy.Duration(2):
	    object_key =  min[0]
    else:
        timestamp = None
    pass

def main(args):

    rospy.init_node('art_pick_and_place')
    point_sub = rospy.Subscriber('/pointing_point', PoseStamped, point_cb)
    global object_key
    obj_sub = rospy.Subscriber('/art_object_detector/object_filtered', InstancesArray, obj_cb)
    state = NOTHING
    pick_and_place = PickAndPlace()
    holding_obj_id = None
    rospy.loginfo("Art pick and place node initialized")
    try:

        while not rospy.is_shutdown():
            #print state
            if state == NOTHING:
                #obj_id = rospy.wait_for_message("/object_to_pick", String)
                if object_key is not None:
		
                    
                    if object_key in objects.keys():
                        holding_obj_id = object_key
                        pick_and_place.pick(objects[holding_obj_id])
                        state = PICKED
                    object_key = None
            elif state == PICKED:
                if holding_obj_id is not None:
                    place_to_put = rospy.wait_for_message("/place_to_put", PoseStamped)
                    print place_to_put

                    pick_and_place.place(place_to_put, objects[holding_obj_id])

                state = NOTHING
    except KeyboardInterrupt:
        print("Shutting down")
        sys.exit()


if __name__ == '__main__':
    main(sys.argv)