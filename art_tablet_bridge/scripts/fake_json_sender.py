#!/usr/bin/env python
import rospy
from art_msgs.msg import ObjInstance, InstancesArray
from geometry_msgs.msg import Pose, Point, Quaternion
from std_msgs.msg import String
import sys


if __name__ == '__main__':
    rospy.init_node('fake_sender')

    pub = rospy.Publisher('/objects_string', String, queue_size=1, latch=True)
    data = String()
    data.data = '''[
    {
        "name": "tea",
        "position": {
            "x": 3,
            "y": 3,
            "z": 0
        },
        "rotation": {
            "x": 0,
            "y": 0,
            "z": 0
        }
    },
    {
        "name": "case",
        "position": {
            "x": 25,
            "y": 5,
            "z": 0
        },
        "rotation": {
            "x": 0,
            "y": 0,
            "z": 0
        }
    },
    {
        "name": "juice",
        "position": {
            "x": 5,
            "y": 18,
            "z": 0
        },
        "rotation": {
            "x": 0,
            "y": 0,
            "z": 0
        }
    }
]'''
    try:
        while not rospy.is_shutdown():
            pub.publish(data)
    except rospy.ROSInterruptException:
        sys.exit()
