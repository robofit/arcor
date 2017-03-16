#!/usr/bin/env python

import rospy
import numpy as np

import pycopia.OS.Linux.Input as input

from geometry_msgs.msg import PointStamped
from art_msgs.msg import Touch
from art_msgs.srv import TouchCalibrationPoints, TouchCalibrationPointsRequest
from std_srvs.srv import Empty as EmptySrv, EmptyResponse
from std_msgs.msg import Bool, Empty
from copy import deepcopy
import cv2
import ast


class Slot:

    def __init__(self, slot_id=None, track_id=None):
        if slot_id is None:
            self.slot_id = -1
        else:
            self.slot_id = slot_id

        if track_id is None:
            self.track_id = -1
        else:
            self.track_id = track_id

        self.x = 0
        self.y = 0

    def __eq__(self, other):
        return self.slot_id == other.slot_id


class ArtTouchDriver:

    def __init__(self):
        self.x = 0
        self.y = 0
        self.touch = False
        self.touch_id = -1
        self.device = input.EventDevice("/dev/input/event18")

        self.slots = []
        self.slot = None
        self.to_delete_id = -1

        self.ns = '/art/interface/touchtable/'

        self.touch_pub = rospy.Publisher(self.ns + "touch", Touch, queue_size=100,  tcp_nodelay=True)  # make sure that all messages will be sent
        self.calibrated_pub = rospy.Publisher(self.ns + 'calibrated', Bool, queue_size=1, latch=True)
        self.calibrating_pub = rospy.Publisher(self.ns + 'calibrating', Bool, queue_size=1, latch=True)
        self.touch_det_pub = rospy.Publisher(self.ns + 'touch_detected', Empty, queue_size=10)
        self.calibrate_req_srv = rospy.Service(self.ns + "calibrate", EmptySrv, self.calibrate_req_srv_cb)

        self.calib_srv = rospy.ServiceProxy('/art/interface/projected_gui/touch_calibration', TouchCalibrationPoints)

        self.set_calibrated(False)
        self.set_calibrating(False)

        self.h_matrix = rospy.get_param('~calibration_matrix', None)

        if self.h_matrix is not None:
            rospy.loginfo("Loaded calibration from param server")
            self.h_matrix = np.matrix(ast.literal_eval(self.h_matrix))
            self.set_calibrated(True)

    def set_calibrated(self, state):

        self.calibrated = state
        self.calibrated_pub.publish(self.calibrated)

    def set_calibrating(self, state):

        self.calibrating = state
        self.calibrating_pub.publish(self.calibrating)

    def calibrate_req_srv_cb(self, req):

        rospy.wait_for_service('/art/interface/projected_gui/touch_calibration')  # TODO wait in __init__??

        req = TouchCalibrationPointsRequest()
        ps = PointStamped()
        ps.header.stamp = rospy.Time.now()
        ps.header.frame_id = "marker"
        ps.point.z = 0

        self.ref_points = ((0.4, 0.1), (1.0, 0.1), (0.4, 0.5), (1.0, 0.5))

        for pt in self.ref_points:

            ps.point.x = pt[0]
            ps.point.y = pt[1]
            req.points.append(deepcopy(ps))

        try:
            resp = self.calib_srv(req)
        except rospy.ServiceException, e:
            print "Service call failed: %s" % e
            self.set_calibrating(False)
            return EmptyResponse()

        if resp.success:

            self.touch_cnt = 0
            self.calib_points = []
            self.set_calibrating(True)
            rospy.loginfo('Starting calibration')

        else:

            self.set_calibrating(False)
            rospy.logerr('Failed to start calibration')

        return EmptyResponse()

    def get_slot_by_id(self, slot_id):
        for slot in self.slots:
            if slot.slot_id == slot_id:
                return slot
        return None

    def process(self):
        # print 1 if self.device._eventq else 0
        event = self.device.read()
        while True:
            if event.evtype == 3 and event.code == 47 and event.value >= 0:
                # MT_SLOT
                self.slot = self.get_slot_by_id(event.value)
                if self.slot is None:
                    self.slot = Slot(slot_id=event.value)
                    self.slots.append(self.slot)

            elif event.evtype == 3 and event.code == 57 and event.value >= 0:
                # MT_TRACK_ID start
                if self.slot is None:
                    self.slot = Slot(track_id=event.value, slot_id=0)
                    self.slots.append(self.slot)
                else:
                    self.slot.track_id = event.value

            elif event.evtype == 3 and event.code == 57 and event.value < 0:
                # MT_TRACK_ID end
                if self.slot is not None:
                    self.to_delete_id = self.slot.track_id
                    self.slots.remove(self.slot)
                    self.slot = None

            elif event.evtype == 3 and event.code == 53:
                # x position
                self.slot.x = event.value

            elif event.evtype == 3 and event.code == 54:
                # y position
                self.slot.y = event.value

            elif event.evtype == 0:

                touch = Touch()
                if self.slot is None:
                    touch.id = self.to_delete_id
                    touch.touch = False
                else:
                    touch.touch = True
                    touch.id = self.slot.track_id
                    touch.point = PointStamped()

                    if self.calibrated:

                        pt = [self.slot.x, self.slot.y, 1]

                        pt = self.h_matrix.dot(np.array(pt, dtype='float64')).tolist()
                        # print pt
                        touch.point.point.x = pt[0][0]
                        touch.point.point.y = pt[0][1]

                    if self.calibrating:

                        # TODO check for "double click" (calc distance from prev touch?)
                        if self.touch_cnt < 4:

                            self.calib_points.append((self.slot.x,  self.slot.y))
                            self.touch_det_pub.publish()
                            self.touch_cnt += 1

                            if self.touch_cnt == 4:

                                self.calculate_calibration()
                                self.set_calibrating(False)

                if self.calibrated:
                    self.touch_pub.publish(touch)

            else:
                pass
            # print event

            if not self.device._eventq:
                break
            event = self.device.read()

    def calculate_calibration(self):

        # print self.calib_points
        # print self.ref_points

        h, status = cv2.findHomography(np.array(self.calib_points, dtype='float64'), np.array(self.ref_points, dtype='float64'))
        self.h_matrix = np.matrix(h)

        s = str(self.h_matrix.tolist())
        rospy.set_param("~calibration_matrix", s)

        # print self.h_matrix

        self.set_calibrated(True)

if __name__ == '__main__':
    rospy.init_node('art_touch_driver')

    rospy.loginfo('Waiting for other nodes to come up...')

    rospy.loginfo('Ready!')

    try:
        node = ArtTouchDriver()
        rate = rospy.Rate(1000)

        while not rospy.is_shutdown():
            node.process()
            # rate.sleep()
    except rospy.ROSInterruptException:
        pass
