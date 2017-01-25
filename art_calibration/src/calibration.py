#!/usr/bin/env python

import numpy as np
from art_utils import ArtCalibrationHelper
from tf import TransformBroadcaster, transformations
import rospy
from geometry_msgs.msg import Transform
from art_calibration import ArtRobotCalibration, ArtCellCalibration
from std_msgs.msg import Bool


class ArtCalibration(object):

    def __init__(self):
        self.robot_calibration = ArtRobotCalibration('pr2', '/pr2/markers',
                                                     '/odom_combined', '/marker')
        self.cells = [ArtCellCalibration('table', '/table/markers',
                                         '/kinect2_link', '/marker')]

        self.calibrated_pub = rospy.Publisher('system_calibrated', Bool,
                                              queue_size=10, latch=True)
        self.calibrated = Bool()
        self.calibrated.data = False
        self.calibrated_pub.publish(self.calibrated)

        self.broadcaster = TransformBroadcaster()

    def publish_calibration(self):
        calibrated = True
        if self.robot_calibration.calibrated:
            tr = self.robot_calibration.get_transform()
            self.broadcaster.sendTransform(tr.translation, tr.rotation,
                                           rospy.Time.now(), self.robot_calibration.world_frame,
                                           self.robot_calibration.robot_frame)
        else:
            calibrated = False

        for cell in self.cells:
            if cell.calibrated:
                tr = cell.get_transform()
                self.broadcaster.sendTransform(tr.translation, tr.rotation,
                                               rospy.Time.now(), self.robot_calibration.world_frame,
                                               self.robot_calibration.robot_frame)
            else:
                calibrated = False

        if calibrated:
            self.calibrated.data = True
            self.calibrated_pub.publish(self.calibrated)

if __name__ == '__main__':
    rospy.init_node('art_calibration', log_level=rospy.INFO)

    try:
        node = ArtCalibration()

        node.calibrate()
        rate = rospy.Rate(30)

        while not rospy.is_shutdown():
            node.publish_calibration()
            rate.sleep()
    except rospy.ROSInterruptException:
        pass
