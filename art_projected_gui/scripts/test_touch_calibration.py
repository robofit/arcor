#!/usr/bin/env python

import sys
import rospy
from art_msgs.srv import TouchCalibrationPoints,  TouchCalibrationPointsRequest
from std_msgs.msg import Bool, Empty
from geometry_msgs.msg import PointStamped
from copy import deepcopy


class TestTouchCalibration():

    def __init__(self):

        # TODO /art/interface/touchtable/active_area param

        self.ns = '/art/interface/touchtable/'
        self.calib_srv = rospy.ServiceProxy('/art/interface/projected_gui/touch_calibration', TouchCalibrationPoints)
        self.calibrated_pub = rospy.Publisher(self.ns + 'calibrated', Bool, queue_size=1,  latch=True)
        self.calibrating_pub = rospy.Publisher(self.ns + 'calibrating', Bool, queue_size=1,  latch=True)
        self.touch_pub = rospy.Publisher(self.ns + 'touch_detected', Empty, queue_size=10)

        self.calibrated_pub.publish(False)
        self.calibrating_pub.publish(False)

        rospy.wait_for_service('/art/interface/projected_gui/touch_calibration')

        req = TouchCalibrationPointsRequest()
        ps = PointStamped()
        ps.header.stamp = rospy.Time.now()
        ps.header.frame_id = "marker"
        ps.point.z = 0

        ps.point.x = 0.2
        ps.point.y = 0.2
        req.points.append(deepcopy(ps))

        ps.point.x = 1.0
        ps.point.y = 0.2
        req.points.append(deepcopy(ps))

        ps.point.x = 0.2
        ps.point.y = 0.6
        req.points.append(deepcopy(ps))

        ps.point.x = 1.0
        ps.point.y = 0.6
        req.points.append(deepcopy(ps))

        try:
            resp = self.calib_srv(req)
        except rospy.ServiceException, e:
            print "Service call failed: %s" % e

        if resp.success:

            self.touch_cnt = 0
            self.tmr = rospy.Timer(rospy.Duration(5), self.tmr_cb)
            self.calibrating_pub.publish(True)
            rospy.loginfo('Starting calibration')

        else:

            rospy.logerr('Failed to start calibration')

    def tmr_cb(self,  evt):

        if self.touch_cnt < 4:

            self.touch_pub.publish()
            self.touch_cnt += 1
            rospy.loginfo('Publishing point no.: ' + str(self.touch_cnt))

        else:
            self.calibrating_pub.publish(False)
            self.calibrated_pub.publish(True)
            self.tmr.shutdown()
            rospy.loginfo('Calibration done')


def main(args):

    rospy.init_node('test_touch_calibration')
    TestTouchCalibration()
    rospy.spin()

if __name__ == '__main__':

    try:
        main(sys.argv)
    except KeyboardInterrupt:
        print("Shutting down")
