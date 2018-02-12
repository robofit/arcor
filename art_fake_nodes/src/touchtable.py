#!/usr/bin/env python

import rospy
from actionlib import SimpleActionServer
from art_msgs.msg import ObjInstance, InstancesArray
from geometry_msgs.msg import Pose
from std_msgs.msg import Header, Bool
import time
import random
from std_srvs.srv import Empty, EmptyResponse


class FakeTouchTable:

    def __init__(self):
        rospy.Service("/art/interface/touchtable/calibrate", Empty, self.calibrated_cb)
        self.calibrated_pub = rospy.Publisher("/art/interface/touchtable/calibrated", Bool, latch=True, queue_size=1)

    def calibrated_cb(self, req):
        r = Bool()
        r.data = True
        self.calibrated_pub.publish(r)
        return EmptyResponse()


if __name__ == '__main__':
    rospy.init_node('fake_touchtable')
    ''',log_level=rospy.DEBUG'''

    try:
        node = FakeTouchTable()
        rate = rospy.Rate(1)
        while not rospy.is_shutdown():
            rate.sleep()
    except rospy.ROSInterruptException:
        pass
