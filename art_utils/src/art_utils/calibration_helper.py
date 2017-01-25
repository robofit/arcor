#!/usr/bin/env python

import numpy as np
from tf import transformations, TransformBroadcaster


class ArtCalibrationHelper(object):

    def __init__(self):
        pass

    @staticmethod
    def transform_from_markers_positions(p0, p1, p2, p3):
        """

        Args:
            p0:
            p1:
            p2:
            p3:

        Returns:

        """

        #      Markers on the table
        #
        #
        #             Robot
        #    11 ------------------12
        #    |                     |
        #    |                     |
        #    |                     |
        #    |                     |
        #    10-------------------13
        #

        pp01 = p1 - p0
        pp03 = p3 - p0

        n = np.cross(pp03, pp01)
        m = np.matrix([[pp03[0], pp01[0], n[0]],
                       [pp03[1], pp01[1], n[1]],
                       [pp03[2], pp01[2], n[2]]])

        return p0, m

    @staticmethod
    def get_marker_position_by_id(markers, marker_id):
        for marker in markers:
            if marker.id == marker_id:
                pos = np.array([0, 0, 0])
                pos[0] = marker.pose.pose.position.x
                pos[1] = marker.pose.pose.position.y
                pos[2] = marker.pose.pose.position.z
                return pos
        return None

