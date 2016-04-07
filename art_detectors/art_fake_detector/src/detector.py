#!/usr/bin/env python

from art_msgs.msg import ObjInstance, InstancesArray
from shape_msgs.msg import SolidPrimitive
import sys
import rospy
import time


class ArCodeDetector:

    def __init__(self):

        self.detected_objects_pub = rospy.Publisher("/art_object_detector/object", InstancesArray, queue_size=10)

        while not rospy.is_shutdown():
            instances = InstancesArray()

            obj_in = ObjInstance()
            obj_in.object_id = str('box')
            obj_in.pose.position.x = 10
            obj_in.pose.position.y = 10
            obj_in.pose.position.z = 0
            obj_in.pose.orientation.x = 0
            obj_in.pose.orientation.y = 0
            obj_in.pose.orientation.z = 0
            obj_in.pose.orientation.w = 1.0
            obj_in.bbox.dimensions = [0.05, 0.05, 0.1]
            obj_in.bbox.type = SolidPrimitive.BOX

            instances.header.frame_id = "/marker"

            instances.instances.append(obj_in)
            self.detected_objects_pub.publish(instances)
            time.sleep(1)

            pass


if __name__ == '__main__':
    rospy.init_node('art_fake_detector')
    try:
        node = ArCodeDetector()
    except rospy.ROSInterruptException:
        pass
