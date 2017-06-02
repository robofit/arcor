#!/usr/bin/env python

import rospy
from art_msgs.msg import ObjInstance, InstancesArray
import time
import sys
import random


class FakeDetector:
    def __init__(self, obj_id, frame_id, pos, noise):
        self.object_publisher = rospy.Publisher('/art/object_detector/object',
                                                InstancesArray, queue_size=10, latch=True)

        self.frame_id = frame_id
        self.pos = pos
        self.noise = noise

        self.obj = ObjInstance()
        self.obj.object_id = obj_id
        self.obj.object_type = "profile_20_60"

        self.obj.pose.orientation.x = 0
        self.obj.pose.orientation.y = 0
        self.obj.pose.orientation.z = 0
        self.obj.pose.orientation.w = 1

        self.timer = rospy.Timer(rospy.Duration(0.1), self.timer_callback)

    def timer_callback(self, evt):
        ia = InstancesArray()

        ia.header.stamp = rospy.Time.now()
        ia.header.frame_id = self.frame_id

        self.obj.pose.position.x = self.pos[0] + random.uniform(-self.noise, self.noise)
        self.obj.pose.position.y = self.pos[1] + random.uniform(-self.noise, self.noise)
        self.obj.pose.position.z = self.pos[2] + random.uniform(-self.noise, self.noise)

        ia.instances = [self.obj]

        self.object_publisher.publish(ia)


if __name__ == '__main__':

    rospy.init_node('fake_detector', anonymous=True)

    try:
        pos = (float(sys.argv[3]), float(sys.argv[4]), float(sys.argv[5]))
        FakeDetector(sys.argv[1], sys.argv[2], pos, float(sys.argv[6]))
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
    except IndexError:
        print "Arguments: obj_id frame_id x y z noise"
