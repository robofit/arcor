#!/usr/bin/env python

import rospy
from actionlib import SimpleActionServer
from art_msgs.msg import ObjInstance, InstancesArray
from geometry_msgs.msg import Pose
from std_msgs.msg import Header
import time
import random


class FakeObjectTracker:

    def __init__(self):
        self.object_publisher = rospy.Publisher('/art/object_detector/object_filtered',
                                                InstancesArray, queue_size=10, latch=True)

    def publish_objects(self):
        obj = ObjInstance()
        obj.object_id = "profile_20_60"
        obj.object_type = "profile"
        obj.pose = Pose()
        obj.pose.position.x = 1
        obj.pose.position.y = 1
        obj.pose.position.z = 0
        obj.pose.orientation.x = 0
        obj.pose.orientation.y = 0
        obj.pose.orientation.z = 0
        obj.pose.orientation.w = 1
        objs = InstancesArray()
        objs.instances.append(obj)
        objs.header = Header()
        self.object_publisher.publish(objs)

if __name__ == '__main__':
    rospy.init_node('fake_tracker')
    ''',log_level=rospy.DEBUG'''

    try:
        node = FakeObjectTracker()
        rate = rospy.Rate(1)
        while not rospy.is_shutdown():
            node.publish_objects()
            rate.sleep()
    except rospy.ROSInterruptException:
        pass
