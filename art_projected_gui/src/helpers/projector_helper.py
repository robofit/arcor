#!/usr/bin/env python

import rospy
from std_msgs.msg import Bool
from std_srvs.srv import Empty

class ProjectorHelper():

    def __init__(self,  proj_id):

        self.proj_id = proj_id
        self.calibrated = False
        self.calibrated_cb = None
        self.calibrating = False

        proj_ns = "/art/interface/projected_gui/projector/" + proj_id + "/"

        self.calib_sub = rospy.Subscriber(proj_ns + "calibrated",  Bool,  self.calib_cb,  queue_size=10)
        self.srv_calibrate = rospy.ServiceProxy(proj_ns + "calibrate", Empty)

    def wait_until_available(self):

        self.srv_calibrate.wait_for_service()

    def calibrate(self,  calibrated_cb = None):

        if self.calibrating:
            return False

        try:
            self.srv_calibrate()
        except rospy.ServiceException:
            return False
        self.calibrating = True
        self.calibrated_cb = calibrated_cb
        return True

    def is_calibrated(self):

        return self.calibrated

    def calib_cb(self,  msg):

        self.calibrated = msg.data
        self.calibrating = False
        if self.calibrated_cb is not None: self.calibrated_cb(self)
