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

        self.objects = []
        obj = ObjInstance()
        obj.object_id = "profile_20_60"
        obj.object_type = "profile_20_60"

        obj.pose = Pose()
        obj.pose.position.x = 0.3
        obj.pose.position.y = 0.3
        obj.pose.position.z = 0
        obj.pose.orientation.x = 0
        obj.pose.orientation.y = 0
        obj.pose.orientation.z = 0
        obj.pose.orientation.w = 1
        self.objects.append(obj)
        obj = ObjInstance()
        obj.object_id = "profile_21_60"
        obj.object_type = "profile_20_60"

        obj.pose = Pose()
        obj.pose.position.x = 0.75
        obj.pose.position.y = 0.58
        obj.pose.position.z = 0
        obj.pose.orientation.x = 0
        obj.pose.orientation.y = 0
        obj.pose.orientation.z = 0
        obj.pose.orientation.w = 1
        self.objects.append(obj)

    def publish_objects(self):

        objs = InstancesArray()
        for obj in self.objects:
            objs.instances.append(obj)
        objs.header = Header()
        objs.header.frame_id = "marker"
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
