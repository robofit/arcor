#!/usr/bin/env python
import rospy
from art_object_recognizer_msgs.msg import ObjInstance, InstancesArray
from geometry_msgs.msg import Pose, Point, Quaternion

if __name__ == '__main__':
    rospy.init_node('fake_sender')

    pub = rospy.Publisher('/art_object_detector/object', InstancesArray, queue_size=1, latch=True)
    objects = InstancesArray()
    objects.instances.append(ObjInstance('obj1', Pose(Point(0, 1, 2), Quaternion(0, 0, 0, 1))))
    objects.instances.append(ObjInstance('obj2', Pose(Point(1, 2, 4), Quaternion(0, 0, 0, 1))))
    objects.instances.append(ObjInstance('obj3', Pose(Point(2, 3, 5), Quaternion(0, 0, 0, 1))))
    pub.publish(objects)
    rospy.spin()
