#!/usr/bin/env python

import rospy
from actionlib import SimpleActionServer
from art_msgs.msg import ObjInstance, InstancesArray
from geometry_msgs.msg import Pose
from std_msgs.msg import Header, Bool
from std_srvs.srv import Empty, EmptyResponse
import time
import random


class FakeCalibration:

    calib_table = False

    def __init__(self):
        self.touch_calib_srv = rospy.Service('/art/interface/touchtable/calibrate',
                                             Empty, self.srv_touch_calib_cb)

        self.touch_calibrating_pub = rospy.Publisher('/art/interface/touchtable/calibrating',
                                                     Bool, queue_size=10, latch=True)

        self.touch_calibrated_pub = rospy.Publisher('/art/interface/touchtable/calibrated',
                                                    Bool, queue_size=10, latch=True)

        self.system_calibrated = rospy.Publisher('/system_calibrated',
                                                 Bool, queue_size=10, latch=True)

        tr, fa = Bool(), Bool()
        tr.data = True
        fa.data = False
        self.touch_calibrated_pub.publish(fa)
        self.touch_calibrating_pub.publish(fa)
        self.system_calibrated.publish(tr)

        self.attempts = 0

    def process(self):
        if not self.calib_table:
            return
        tr, fa = Bool(), Bool()
        tr.data = True
        fa.data = False
        self.touch_calibrating_pub.publish(tr)
        rospy.sleep(2)
        self.attempts += 1
        if self.attempts >= 3:
            self.touch_calibrated_pub.publish(tr)
        else:
            self.touch_calibrated_pub.publish(fa)
        self.touch_calibrating_pub.publish(fa)
        self.calib_table = False

    def srv_touch_calib_cb(self, req):
        self.calib_table = True
        return EmptyResponse()

if __name__ == '__main__':
    rospy.init_node('fake_calibration')
    ''',log_level=rospy.DEBUG'''

    try:
        node = FakeCalibration()
        rate = rospy.Rate(30)
        while not rospy.is_shutdown():
            node.process()
            rate.sleep()
    except rospy.ROSInterruptException:
        pass
