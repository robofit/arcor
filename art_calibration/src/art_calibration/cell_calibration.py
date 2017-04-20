#!/usr/bin/env python

import numpy as np
from art_utils import ArtCalibrationHelper
from tf import TransformBroadcaster, transformations
import rospy
from geometry_msgs.msg import Transform
from ar_track_alvar_msgs.msg import AlvarMarker, AlvarMarkers



class ArtCellCalibration(object):

    def __init__(self, cell_id, markers_topic, world_frame,  cell_frame):
        self.cell_id = cell_id
        self.markers_topic = markers_topic
        self.calibrated = None
        self.positions = [None, None, None, None]

        self.cell_frame = cell_frame
        self.world_frame = world_frame
        self.transformation = Transform()

        self.markers_sub = rospy.Subscriber(self.markers_topic, AlvarMarkers, self.markers_cb,  queue_size=1)
        rospy.loginfo("Cell: " + str(self.cell_id) + " ready")

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
            return 

        self.transformation.rotation = ArtCalibrationHelper.normalize_vector(transformations.quaternion_from_matrix(m))
        self.transformation.translation = transformations.translation_from_matrix(m)
        self.calibrated = True

    def get_transform(self):
        if not self.calibrated:
            
            return None
        
        return self.transformation

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
            self.calibrate()

