#!/usr/bin/env python

import rospy
import json
from ws4py.client.threadedclient import WebSocketClient
from geometry_msgs.msg import PoseStamped
import collections
import numpy as np
import socket
import tf
from math import sqrt

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

        self.frame_id = frame_id

        self.left_filt = [0,  0 , 0] # left hand - filtered data
        self.right_filt = [0,  0 , 0] # right hand - filtered data

        self.left_orientation = [0,  0,  0,  1] # TODO also do some filtering?
        self.right_orientation = [0,  0,  0,  1]

        self.left_ts = None
        self.right_ts = None

        self.left_pub = rospy.Publisher("leap/palm/left", PoseStamped, queue_size=1)
        self.right_pub = rospy.Publisher("leap/palm/right", PoseStamped, queue_size=1)

        self.tmr = rospy.Timer(rospy.Duration(1.0/30), self.tmr_callback)

    def publish(self,  pos,  orientation,  pub):

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

        pub.publish(ps)

    def tmr_callback(self,  evt):

        if self.left_ts is not None and rospy.Time.now() - self.left_ts < rospy.Duration(1): self.publish(self.left_filt, self.left_orientation,   self.left_pub)
        if self.right_ts is not None and rospy.Time.now() - self.right_ts < rospy.Duration(1): self.publish(self.right_filt,  self.right_orientation,  self.right_pub)

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

        # TODO average orientation
        # for now - just take the first one
        if len(left) > 0: self.left_orientation = left[0].orientation
        if len(right) > 0: self.right_orientation = right[0].orientation

        if left_avg is not None:

            for i in range(0, 3):

                self.left_filt[i] = fa*self.left_filt[i] + (1-fa)*left_avg[i]

        if right_avg is not None:

            for i in range(0, 3):

                self.right_filt[i] = fa*self.right_filt[i] + (1-fa)*right_avg[i]

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

                # apply sensor orientation in 'world' frame
                h.pos = self.qv_mult(self.orientation,  h.pos)

                # apply sensor position in 'world' frame
                h.pos[0] += self.pos[0]
                h.pos[1] += self.pos[1]
                h.pos[2] += self.pos[2]

                # TODO convert normal vector (palmNormal) to orientation -> fix it
                #o = []
                #o.append(frame['palmNormal'][0])
                #o.append(-frame['palmNormal'][2])
                #o.append(frame['palmNormal'][1])

                #axis = np.array([0,  0,  1.0])

                #o = np.array(o)

                #m = sqrt(2.0 + 2.0 * np.dot(o, axis))
                #w = (1.0 / m) * np.cross(o, axis)
                #q = [w[0], w[1], w[2],  0.5 * m]

                #h.orientation = q
                h.orientation = [0,  0,  0,  1]

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

    r = rospy.Rate(100)

    while not rospy.is_shutdown():

        c.process()
        r.sleep()

    c.stop()

if __name__ == '__main__':

    try:
        main()
    except KeyboardInterrupt:
        pass


