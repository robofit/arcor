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
        self.robot_calibration = ArtRobotCalibration('pr2', '/pr2/ar_pose_marker',
                                                     '/odom_combined', '/marker_detected')

        self.cells = []

        for cell in rospy.get_param("cells", ["n1","n2"]):

            self.cells.append(ArtCellCalibration(cell, '/art/' + cell + '/ar_pose_marker',
                                                 '/' + cell + '_kinect2_link', '/marker_detected'))

        self.calibrated_pub = rospy.Publisher('system_calibrated', Bool,
                                              queue_size=10, latch=True)
        self.calibrated = Bool()
        self.calibrated.data = False
        self.calibrated_sended = False
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
            calibrated = True

        for cell in self.cells:
            if cell.calibrated:
                tr = cell.get_transform()
                self.broadcaster.sendTransform(tr.translation, tr.rotation,
                                               rospy.Time.now(), cell.world_frame,
                                               cell.cell_frame)

            else:
                calibrated = False

        if calibrated and not self.calibrated_sended:
            self.calibrated_sended = True
            self.calibrated.data = True
            self.calibrated_pub.publish(self.calibrated)


if __name__ == '__main__':
    rospy.init_node('art_calibration', log_level=rospy.INFO)

    try:
        node = ArtCalibration()

        rate = rospy.Rate(30)

        while not rospy.is_shutdown():
            node.publish_calibration()
            rate.sleep()
    except rospy.ROSInterruptException:
        pass
