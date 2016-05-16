#!/usr/bin/env python

import rospy
import time

from leap_motion.msg import leapros
from visualization_msgs.msg import Marker
from tf import transformations
import tf
import tf2_geometry_msgs
import tf2_ros
from geometry_msgs.msg import Vector3, TransformStamped, Point
import copy
import numpy as np
import scipy
import cv2


class TableLeap:
    def __init__(self):
        self.leap_sub = rospy.Subscriber("/leapmotion/data", leapros, self.leap_data_cb, queue_size=1)
        self.markers_pub = rospy.Publisher("/table_pointing_leap/visualize", Marker, queue_size=1)
        self.table_frame = "/table"
        self.leap_data = None # leapros
        self.h_matrix = None
        self.calibrate()
        pass

    def leap_data_cb(self, data):
        '''

        :type data: leapros
        :return:
        '''

        self.leap_data = data

        if self.h_matrix is None:
            return
        plane = np.array([0, 0, 0])
        plane_normal = np.array([0, 0, 1])
        intersection = self.compute_intersection(self.transform_leap_data(data.index_distal),
                                                 self.transform_leap_data(data.index_tip),
                                                 plane,
                                                 plane_normal)

        # print self.h_matrix.dot(intersection[0:2])
        #intersection = intersection[0:2]
        print intersection
        res = self.h_matrix.dot(intersection)
        print res
        print ""
        return

        position, orientation = self.transform_leap_data(data.palmpos,
                                                         data.ypr)

        orientation_q = transformations.quaternion_from_euler(orientation.x, orientation.y, orientation.z)
        # self.show_arrow(position, orientation_q)
        self.show_box(self.transform_leap_data(data.thumb_tip), 1)
        self.show_box(self.transform_leap_data(data.index_tip), 2)
        self.show_box(self.transform_leap_data(data.index_distal), 6)
        self.show_box(self.transform_leap_data(data.index_intermediate), 7)
        self.show_box(self.transform_leap_data(data.middle_tip), 3)
        self.show_box(self.transform_leap_data(data.ring_tip), 4)
        self.show_box(self.transform_leap_data(data.pinky_tip), 5)
        plane = np.array([0, 0, 0])
        plane_normal = np.array([0, 0, 1])
        intersection = self.compute_intersection(self.transform_leap_data(data.index_distal),
                                                 self.transform_leap_data(data.index_tip),
                                                 plane,
                                                 plane_normal)
        self.show_arrow_from_two_points(self.transform_leap_data(data.index_distal),
                                        intersection, 22)

        intersection2 = self.compute_intersection(self.transform_leap_data(data.index_intermediate),
                                                  self.transform_leap_data(data.index_tip),
                                                  plane,
                                                  plane_normal)
        self.show_arrow_from_two_points(self.transform_leap_data(data.index_intermediate),
                                        intersection2, 23)

        pass

    def calibrate(self):
        points = []
        plane = np.array([0, 0, 0])
        plane_normal = np.array([0, 0, 1])

        print("Reading first point")
        rospy.sleep(2)
        intersection = self.compute_intersection(self.transform_leap_data(self.leap_data.index_distal),
                                                 self.transform_leap_data(self.leap_data.index_tip),
                                                 plane,
                                                 plane_normal)
        points.append(intersection)
        print("Reading second point")
        rospy.sleep(2)
        intersection = self.compute_intersection(self.transform_leap_data(self.leap_data.index_distal),
                                                 self.transform_leap_data(self.leap_data.index_tip),
                                                 plane,
                                                 plane_normal)
        points.append(intersection)
        print("Reading third point")
        rospy.sleep(2)
        intersection = self.compute_intersection(self.transform_leap_data(self.leap_data.index_distal),
                                                 self.transform_leap_data(self.leap_data.index_tip),
                                                 plane,
                                                 plane_normal)
        points.append(intersection)
        print("Reading last point")
        rospy.sleep(2)
        intersection = self.compute_intersection(self.transform_leap_data(self.leap_data.index_distal),
                                                 self.transform_leap_data(self.leap_data.index_tip),
                                                 plane,
                                                 plane_normal)
        points.append(intersection)
        print("Done")

        points = np.array(points)[:, 0:2]
        m_points = np.array([[0.0, 0.0],
                             [0.0, 1.0],
                             [1.0, 1.0],
                             [1.0, 0.0]])
        print m_points.dtype
        print points.dtype

        h, status = cv2.findHomography(np.array(points)[:, 0:2], m_points, cv2.LMEDS)
        self.h_matrix = np.matrix(h)
        print h
        pass

    def transform_leap_data(self, position, orientation=None):
        '''

        :type position: Vector3
        :type orientation: Vector3
        :return:
        '''
        position_transformed = Vector3(float(position.z) / -1000,
                                       float(position.x) / -1000,
                                       float(position.y) / 1000)
        if orientation is None:
            return position_transformed
        orientation_transformed = Vector3(float(-orientation.z * 3.14 / 180),
                                          float(-orientation.x * 3.14 / 180),
                                          float(orientation.y * 3.14 / 180))

        return position_transformed, orientation_transformed

    def compute_intersection(self, point1, point2, plane, plane_normal):
        '''

        :type point1: Vector3
        :type point2: Vector3
        :type plane: np.array()
        :type plane_normal: np.array()
        :return: TransformStamped
        '''
        pp1 = np.array([point1.x, point1.y, point1.z])
        pp2 = np.array([point2.x, point2.y, point2.z])

        l = pp2 - pp1
        d = (plane - pp1).dot(plane_normal)

        d /= l.dot(plane_normal)
        return l * d + pp1

    def show_arrow_from_two_points(self, point1, point2, array_id):
        marker = Marker()
        marker.type = marker.ARROW
        marker.action = marker.ADD
        marker.color.r = 0
        marker.color.g = 1
        marker.color.b = 0
        marker.color.a = 1
        marker.scale.x = 0.01
        marker.scale.y = 0.02
        marker.scale.z = 0
        marker.points.append(Point(point1.x, point1.y, point1.z))
        # marker.points.append(Point(point2.x, point2.y, point2.z))
        marker.points.append(Point(point2[0], point2[1], point2[2]))
        marker.header.frame_id = self.table_frame
        marker.header.stamp = rospy.get_rostime()
        marker.id = array_id

        self.markers_pub.publish(marker)

    def show_arrow(self, position, direction):
        marker = Marker()
        marker.type = marker.ARROW
        marker.action = marker.ADD
        marker.color.r = 1
        marker.color.g = 0
        marker.color.b = 0
        marker.color.a = 1
        marker.scale.x = 1.5
        marker.scale.y = 0.01
        marker.scale.z = 0.01
        marker.pose.position.x = position.x
        marker.pose.position.y = position.y
        marker.pose.position.z = position.z
        marker.pose.orientation.x = direction[0]
        marker.pose.orientation.y = direction[1]
        marker.pose.orientation.z = direction[2]
        marker.pose.orientation.w = direction[3]
        marker.header.frame_id = self.table_frame
        marker.header.stamp = rospy.get_rostime()
        marker.id = 0

        self.markers_pub.publish(marker)

    def show_box(self, position, marker_id):
        marker = Marker()
        marker.type = marker.CUBE
        marker.action = marker.ADD
        marker.color.r = 1
        marker.color.g = 0
        marker.color.b = 0
        marker.color.a = 1
        marker.scale.x = 0.01
        marker.scale.y = 0.01
        marker.scale.z = 0.01
        marker.pose.position.x = position.x
        marker.pose.position.y = position.y
        marker.pose.position.z = position.z
        marker.pose.orientation.x = 0
        marker.pose.orientation.y = 0
        marker.pose.orientation.z = 0
        marker.pose.orientation.w = 1
        marker.header.frame_id = self.table_frame
        marker.header.stamp = rospy.get_rostime()
        marker.id = marker_id

        self.markers_pub.publish(marker)


if __name__ == '__main__':
    rospy.init_node('art_table_pointing_leap_node')
    try:
        node = TableLeap()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
