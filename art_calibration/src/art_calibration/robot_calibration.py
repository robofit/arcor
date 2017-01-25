#!/usr/bin/env python

import numpy as np
from art_utils import ArtCalibrationHelper
from tf import TransformBroadcaster, transformations
import rospy
from geometry_msgs.msg import Transform, PointStamped
from ar_track_alvar_msgs.msg import AlvarMarker, AlvarMarkers


class ArtRobotCalibration(object):

    def __init__(self, robot_id, markers_topic, robot_frame, world_frame):
        self.cell_id = robot_id
        self.markers_topic = markers_topic
        self.calibrated = None
        self.positions = [None, None, None, None]
        self.broadcaster = TransformBroadcaster()

        self.robot_frame = robot_frame
        self.world_frame = world_frame

        self.markers_sub = rospy.Subscriber(self.markers_topic, AlvarMarkers, queue_size=1)
        self.head_look_at_pub = rospy.Publisher('/art/pr2/look_at',  PointStamped,  queue_size=1)

        self.robot_state = 0
        self.robot_looking_for_id = 10
        self.count = 0

    def calibrate(self):
        for p in self.positions:
            if p is None:
                return False
        point, m = ArtCalibrationHelper.transform_from_markers_positions(self.positions[0],
                                                                         self.positions[1],
                                                                         self.positions[2],
                                                                         self.positions[3])

        if point is None or m is None:
            return 
            
        self.transformation.rotation = transformations.quaternion_from_matrix(m)
        self.transformation.translation = point
        self.calibrated = True

    def get_transform(self):
        if not self.calibrated:
            return None
        return self.transformation

    def markers_cb(self, markers):
        if self.calibrated:
            return

        point = PointStamped()
        point.header.frame_id = "/base_link"
        point.point.x = 0.3
        point.point.z = 1
        if self.robot_state == 0:
            point.point.y = -0.5
            self.head_look_at_pub.publish(point)
            rospy.sleep(5)
        elif self.robot_state == 1 and self.robot_looking_for_id == 12:
            point.point.y = 0.4
            self.head_look_at_pub.publish(point)
            rospy.sleep(5)
        elif self.robot_state == 2 and self.robot_looking_for_id > 20:
            self.calibrate()

        # look for markers

        if self.robot_looking_for_id < 20:
            p = ArtCalibrationHelper().get_marker_position_by_id(markers, self.robot_looking_for_id)
            if p is not None:
                self.positions[self.robot_looking_for_id-10] += p
                self.count += 1
                if self.count >= 10:
                    self.positions[self.robot_looking_for_id - 10] /= self.count
                    self.count = 0
                    self.robot_looking_for_id += 1
                    if self.robot_looking_for_id > 13:
                        self.robot_looking_for_id += 100

        

