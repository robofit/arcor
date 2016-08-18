#!/usr/bin/env python

import rospy
import json
from ws4py.client.threadedclient import WebSocketClient
from geometry_msgs.msg import PoseStamped,  PointStamped
import collections
import numpy as np
import socket
import tf
from math import atan2,  pi
import pyrr

class HandInfo():

    def __init__(self):

        self.timestamp = None
        self.pos = None
        self.orientation = None
        self.velocity = None
        self.visible = None
        self.confidence = None

class MultiLeapClient():

    def __init__(self, frame_id):

        self.leaps = []

        self.table_min_x = rospy.get_param('~table_min_x')
        self.table_max_x = rospy.get_param('~table_max_x')
        self.table_min_y = rospy.get_param('~table_min_y')
        self.table_max_y = rospy.get_param('~table_max_y')

        self.frame_id = frame_id

        self.left_filt =None # left hand - filtered data
        self.right_filt = None # right hand - filtered data

        self.left_orientation =None
        self.right_orientation = None

        self.left_ts = None
        self.right_ts = None

        self.left_pub = rospy.Publisher("palm/3d/left", PoseStamped, queue_size=1)
        self.right_pub = rospy.Publisher("palm/3d/right", PoseStamped, queue_size=1)

        self.left_2d_pub = rospy.Publisher("palm/2d/left", PointStamped, queue_size=1)
        self.right_2d_pub = rospy.Publisher("palm/2d/right", PointStamped, queue_size=1)

        self.tmr = rospy.Timer(rospy.Duration(1.0/30), self.tmr_callback)

    def publish(self,  pos,  orientation,  pub_3d,  pub_2d):

        if orientation is None or pos is None: return

        orientation = self.normalize(orientation)

        ps = PoseStamped()
        ps.header.stamp = rospy.Time.now()
        ps.header.frame_id = self.frame_id
        ps.pose.position.x = pos[0]
        ps.pose.position.y = pos[1]
        ps.pose.position.z = pos[2]

        ps.pose.orientation.x = orientation[0]
        ps.pose.orientation.y = orientation[1]
        ps.pose.orientation.z = orientation[2]
        ps.pose.orientation.w = orientation[3]

        pub_3d.publish(ps)

        dir = pyrr.quaternion.apply_to_vector(pyrr.quaternion.conjugate(orientation), pos)
        dir[0] += 10.0
        dir = pyrr.quaternion.apply_to_vector(orientation,  dir)

        r = pyrr.ray.create(pos, dir)
        p = pyrr.plane.create( [0,  0,  1])

        intersection = pyrr.geometric_tests.ray_intersect_plane(r,  p,  front_only=True)

        if intersection is not None:

            (x,  y) = self.limit_to_table(intersection[0],  intersection[1])

            pt = PointStamped()
            pt.header = ps.header
            pt.point.x = x
            pt.point.y = y
            pt.point.z = 0.0

            pub_2d.publish(pt)

    def clamp(self,  n, minn, maxn):
        return max(min(maxn, n), minn)

    def limit_to_table(self,  x,  y):

        x = self.clamp(x,  self.table_min_x,  self.table_max_x)
        y = self.clamp(y,  self.table_min_y,  self.table_max_y)
        return (x,  y)

    def tmr_callback(self,  evt):

        if self.left_ts is not None:

            if rospy.Time.now() - self.left_ts < rospy.Duration(0.5):

                self.publish(self.left_filt, self.left_orientation,   self.left_pub,  self.left_2d_pub)

            else:

                self.left_filt = None
                self.left_orientation = None

        if self.right_ts is not None:

            if rospy.Time.now() - self.right_ts < rospy.Duration(0.5):

                self.publish(self.right_filt,  self.right_orientation,  self.right_pub,  self.right_2d_pub)

            else:

                self.right_filt = None
                self.right_orientation = None

    def add(self,  leap):

        self.leaps.append(LeapClient(leap))
        try:
            self.leaps[-1].connect()
        except socket.error:
            rospy.logerr('Could not connect to: ' + leap['conn'] + ' (' + leap['id'] + ')')

    def process(self):

        left = []
        right = []

        for leap in self.leaps:

            try:
                left.append(leap.buff["left"].popleft())
            except IndexError:
                pass

            try:
                right.append(leap.buff["right"].popleft())
            except IndexError:
                pass

        if len(left) > 0: self.left_ts = rospy.Time.now()
        if len(right) > 0: self.right_ts = rospy.Time.now()

        left_avg = self.get_avg(left)
        right_avg = self.get_avg(right)

        fa = 0.2

        lo = self.get_avg_orientation(left)
        ro = self.get_avg_orientation(right)

        self.left_orientation = self.temp_filter(self.left_orientation,  lo,  4,  fa)
        self.right_orientation = self.temp_filter(self.right_orientation,  ro,  4,  fa)
        self.left_filt = self.temp_filter(self.left_filt,  left_avg,  3,  fa)
        self.right_filt = self.temp_filter(self.right_filt,  right_avg,  3,  fa)

    def temp_filter(self,  filt,  data,  l,  coef):

        if data is None: return filt

        if filt is None:

            return data

        else:

            for i in range(0, l):

                filt[i] = coef*filt[i] + (1-coef)*data[i]

            return filt

    def normalize(self, q, tolerance=0.00001):

        n = np.linalg.norm(q)

        if n > tolerance:

            q = q / n
        return q

    def get_avg_orientation(self,  data):

        if len(data) == 0: return None

        res = np.array(data[0].orientation)

        for i in range(1,  len(data)):

            if np.dot(data[i].orientation,  data[0].orientation) < 0.0:

              data[i].orientation = -1.0*np.array(data[i].orientation)

            for j in range(0,  4):

                res[j] += data[i].orientation[j]

        return self.normalize(res / len(data))

    def get_avg(self,  arr):

        if len(arr) == 0: return None

        pos = [[],  [],  []]
        conf = [[],  [],  []]

        for it in arr:

            for i in range(0, 3):

               pos[i].append(it.pos[i])
               conf[i].append(it.confidence)

        return [np.average(pos[0],  weights=conf[0]),  np.average(pos[1],  weights=conf[1]),  np.average(pos[2],  weights=conf[2])]

    def stop(self):

        for leap in self.leaps:

            try:
                leap.close()
            except socket.error:
                pass

class LeapClient(WebSocketClient):

    def __init__(self,  leap):

        super(LeapClient, self).__init__(leap['conn'],  protocols=['http-only', 'chat'])

        self.id = leap['id']
        self.pos = [leap['position']['x'],  leap['position']['y'],  leap['position']['z']]
        self.orientation = tf.transformations.quaternion_from_euler(leap['orientation']['roll'],  leap['orientation']['pitch'],  leap['orientation']['yaw'])

        self.buff = {}
        self.buff["left"] = collections.deque(maxlen=10)
        self.buff["right"] = collections.deque(maxlen=10)

    def set_bool(self,  name,  val):

        cfg = {}
        cfg[name] = val
        self.send(json.dumps(cfg))

    def set_focused(self,  val):

        self.set_bool('focused',  val)

    def enable_background(self,  val):

        self.set_bool('background',  val)

    def enable_gestures(self,  val):

        self.set_bool('gestures',  val)

    def optimize_hmd(self,  val):

        self.set_bool('optimizeHMD',  val)

    def opened(self):

        print "opened"

        self.set_focused(True)
        self.enable_background(True)
        self.enable_gestures(False)
        self.optimize_hmd(False)

    def closed(self, code, reason=None):
        print "Closed down", code, reason

    def received_message(self, m):

        msg = json.loads(str(m))

        if 'serviceVersion' in msg and 'version' in msg:
            if msg['version'] == 6:
                print "version ok"

        elif 'event' in msg:

            if 'state' in msg['event']:

                # attached: true/false, streaming: true/false
                print 'id: ' + msg['event']['state']['id']

        elif 'hands' in msg:

            for frame in msg['hands']:

                #print 'id: ' + str(frame['id']) + ', conf: ' + str(frame['confidence']) + ', pos: ' + str(frame['stabilizedPalmPosition']) + ', vel: ' + str(frame['palmVelocity'])
                # timestamp, palmNormal, palmVelocity, timeVisible, 'stabilizedPalmPosition'

                if frame['confidence'] == 0: continue

                h = HandInfo()
                h.type = frame['type']
                h.pos = PoseStamped()

                h.pos = []
                h.pos.append(0.001*frame['stabilizedPalmPosition'][0]) # TODO metoda na prehazeni os
                h.pos.append(- 0.001*frame['stabilizedPalmPosition'][2])
                h.pos.append(0.001*frame['stabilizedPalmPosition'][1])

                # TODO fix this
                # apply sensor orientation in 'world' frame
                #h.pos = self.qv_mult(self.orientation,  h.pos)

                # apply sensor position in 'world' frame
                h.pos[0] += self.pos[0]
                h.pos[1] += self.pos[1]
                h.pos[2] += self.pos[2]

                # get orientation in leap coordinates
                r = atan2(frame['palmNormal'][0],  -frame['palmNormal'][1])# roll - rotation around the (leap) z-axis
                p = atan2(frame['direction'][1],  -frame['direction'][2]) # pitch represents rotation around the (leap) x-axis
                y = atan2(frame['direction'][0],  -frame['direction'][2]) # yaw - rotation around the (leap) y-axis (ROS z-axis)

                h.orientation = tf.transformations.quaternion_from_euler(-r,  -p,  -y+pi/2)

                # TODO fix following application of sensor orientation
                #h.orientation = tf.transformations.quaternion_multiply(tf.transformations.quaternion_multiply(self.orientation,  h.orientation), tf.transformations.quaternion_conjugate(self.orientation))

                h.velocity = [0.001*frame['palmVelocity'][0],  -0.001*frame['palmVelocity'][2],  0.001*frame['palmVelocity'][1]]

                h.confidence = frame['confidence']
                #h.timestamp = frame['timestamp']

                try:
                    self.buff[frame['type']].append(h)
                except KeyError:
                    print "unknown hand type: " + frame['type']

    # rotate vector v1 by quaternion q1
    def qv_mult(self, q1, v1):
        #v1 = tf.transformations.unit_vector(v1)
        q2 = list(v1)
        q2.append(0.0)

        return tf.transformations.quaternion_multiply(
            tf.transformations.quaternion_multiply(q1, q2),
            tf.transformations.quaternion_conjugate(q1)
        )[:3]

def main():

    rospy.init_node('leap')

    try:
        leaps = rospy.get_param('~leaps')
    except KeyError:
        rospy.logerr('Private parameter "leaps" not set!')
        return

    c = MultiLeapClient(rospy.get_param('~frame_id',  'marker'))

    for leap in leaps:
        c.add(leap)

    rospy.loginfo('ready')

    r = rospy.Rate(200) # max. frame rate (?)

    while not rospy.is_shutdown():

        c.process()
        r.sleep()

    c.stop()

if __name__ == '__main__':

    try:
        main()
    except KeyboardInterrupt:
        pass


