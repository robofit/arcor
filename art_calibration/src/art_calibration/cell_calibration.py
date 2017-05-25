#!/usr/bin/env python

import numpy as np
from art_utils import ArtCalibrationHelper
from tf import TransformBroadcaster, transformations
import rospy
from geometry_msgs.msg import Transform
from ar_track_alvar_msgs.msg import AlvarMarker, AlvarMarkers
from std_msgs.msg import Bool
import ast


class ArtCellCalibration(object):

    def __init__(self, cell_id, markers_topic, world_frame, cell_frame):
        self.cell_id = cell_id
        self.markers_topic = markers_topic
        self.calibrated = None
        self.positions = [None, None, None, None]

        self.cell_frame = cell_frame
        self.world_frame = world_frame
        self.transformation = Transform()

        m = rospy.get_param("~" + self.cell_id + "/calibration_matrix", None)

        self.markers_sub = rospy.Subscriber(self.markers_topic, AlvarMarkers, self.markers_cb, queue_size=1)
        self.marker_detection_enable_publisher = rospy.Publisher("/art/" +
                                                                 self.cell_id +
                                                                 "/ar_track_alvar_bundle_objects/enable_detection",
                                                                 Bool,
                                                                 queue_size=1,
                                                                 latch=True)

        if m is not None:

            m = np.matrix(ast.literal_eval(m))

            self.transformation.rotation = ArtCalibrationHelper.normalize_vector(transformations.quaternion_from_matrix(m))
            self.transformation.translation = transformations.translation_from_matrix(m)
            self.calibrated = True
            rospy.loginfo("Cell: " + str(self.cell_id) + " calibration loaded from param")
            self.stop_marker_detection()

        else:

            self.start_marker_detection()
            rospy.loginfo("Cell: " + str(self.cell_id) + " ready")

    def stop_marker_detection(self):
        detect = Bool()
        detect.data = False
        self.marker_detection_enable_publisher.publish(detect)

    def start_marker_detection(self):
        detect = Bool()
        detect.data = True
        self.marker_detection_enable_publisher.publish(detect)

    def calibrate(self):
        rospy.loginfo("Cell: " + str(self.cell_id) + " trying to calibrate")
        '''for p in self.positions:
            if p is None:
                return False'''
        point, m = ArtCalibrationHelper.transform_from_markers_positions(self.positions[0],
                                                                         self.positions[1],
                                                                         0,
                                                                         self.positions[3])
        if point is None or m is None:
            if point is None:
                rospy.logerr("Origin was not computed!")
            elif m is None:
                rospy.logerr("Transformation matrix was not computed!")
            self.reset_markers_searching()  # let's try to get new positions of markers
            return

        rospy.set_param("~" + self.cell_id + "/calibration_matrix", str(m.tolist()))

        self.transformation.rotation = ArtCalibrationHelper.normalize_vector(transformations.quaternion_from_matrix(m))
        self.transformation.translation = transformations.translation_from_matrix(m)
        self.calibrated = True
        rospy.loginfo("Cell: " + str(self.cell_id) + " calibration done")

    def get_transform(self):
        if not self.calibrated:

            return None

        return self.transformation

    def reset_markers_searching(self):
        self.start_marker_detection()
        self.positions = [None, None, None, None]

    def markers_cb(self, markers):
        if self.calibrated:
            return
        all_markers = True
        for i in xrange(4):
            if self.positions[i] is not None:
                continue
            if i == 2:
                continue
            p = ArtCalibrationHelper.get_marker_position_by_id(markers, i + 10)
            if p is not None:
                self.positions[i] = p
                rospy.loginfo("Cell: " + str(self.cell_id) + " gets marker id " + str(i + 10))

            else:
                all_markers = False
        if all_markers:
            self.stop_marker_detection()
            self.calibrate()
