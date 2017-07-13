#!/usr/bin/env python

import rospy
from art_msgs.msg import ObjInstance, InstancesArray
import time
import sys
import random
from tf import transformations
from math import pi


def a2q(q, arr):

    q.x = arr[0]
    q.y = arr[1]
    q.z = arr[2]
    q.w = arr[3]


class FakeDetector:
    def __init__(self, obj_id, frame_id, pos, rpy, noise):
        self.object_publisher = rospy.Publisher('/art/object_detector/object',
                                                InstancesArray, queue_size=10, latch=True)

        self.frame_id = frame_id
        self.pos = pos
        self.noise = noise

        self.obj = ObjInstance()
        self.obj.object_id = obj_id
        self.obj.object_type = "profile_20_60"

        angles = list(rpy)

        for idx in range(0, len(angles)):

            angles[idx] = angles[idx] / 360.0 * 2 * pi

        # TODO apply noise also to orientation
        a2q(self.obj.pose.orientation, transformations.quaternion_from_euler(*angles))

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
        rpy = (float(sys.argv[6]), float(sys.argv[7]), float(sys.argv[8]))
        FakeDetector(sys.argv[1], sys.argv[2], pos, rpy, float(sys.argv[9]))
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
    except IndexError:
        print "Arguments: obj_id frame_id x y z r p y noise"
