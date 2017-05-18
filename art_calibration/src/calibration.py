#!/usr/bin/env python

import numpy as np
from art_utils import ArtCalibrationHelper
from tf import TransformBroadcaster, transformations
import rospy
from geometry_msgs.msg import Transform
from art_calibration import ArtRobotCalibration, ArtCellCalibration
from std_msgs.msg import Bool
from art_msgs.srv import RecalibrateCell, RecalibrateCellRequest, RecalibrateCellResponse


class ArtCalibration(object):

    def __init__(self):

        self.cells = []

        self.cells.append(ArtRobotCalibration('pr2', '/pr2/ar_pose_marker',
                                              '/odom_combined', '/marker_detected'))

        for cell in rospy.get_param("cells", ["n1", "n2"]):
            self.cells.append(ArtCellCalibration(cell, '/art/' + cell + '/ar_pose_marker',
                                                 '/' + cell + '_kinect2_link', '/marker_detected'))

        self.calibrated_pub = rospy.Publisher('/art/system/calibrated', Bool,
                                              queue_size=10, latch=True)
        self.calibrated = Bool()
        self.calibrated.data = False
        self.calibrated_sended = False
        self.calibrated_pub.publish(self.calibrated)
        self.recalibrate_cell_service = rospy.Service("/art/system/calibrate_cell", RecalibrateCell,
                                                      self.recalibrate_cell_cb)

        self.broadcaster = TransformBroadcaster()

    def recalibrate_cell_cb(self, req):
        resp = RecalibrateCellResponse()
        resp.success = False
        cell_name = req.cell_name
        for cell in self.cells:
            if cell.cell_name == cell_name:
                cell.reset_markers_searching()
                resp.success = True
                break
        else:
            resp.error = "Unknown cell"
        return resp

    def publish_calibration(self):
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
