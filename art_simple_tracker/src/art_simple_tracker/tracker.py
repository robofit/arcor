#! /usr/bin/env python
import rospy
from art_msgs.msg import InstancesArray, ObjInstance
import tf
from geometry_msgs.msg import Pose, PoseStamped
from math import sqrt
import numpy as np
from scipy.spatial import distance
import threading


# TODO publish TF

def normalize(q, tolerance=0.00001):
    v = (q.x, q.y, q.z, q.w)
    mag2 = sum(n * n for n in v)
    if abs(mag2 - 1.0) > tolerance:
        mag = sqrt(mag2)
        q.x /= mag
        q.y /= mag
        q.z /= mag
        q.w /= mag
    return q


def q2a(q):
    return [q.x, q.y, q.z, q.w]


class TrackedObject:
    def __init__(self, target_frame, tfl, inst):

        self.inst = inst
        self.tfl = tfl
        self.target_frame = target_frame
        self.max_dist = 2.0
        self.min_meas_cnt = 5
        self.new = True

        self.meas = {}

    def add_meas(self, ps):

        dist = distance.euclidean((0, 0, 0), (ps.pose.position.x, ps.pose.position.y, ps.pose.position.z))

        if dist > self.max_dist:
            return

        if ps.header.frame_id not in self.meas:
            self.meas[ps.header.frame_id] = []

        if len(self.meas[ps.header.frame_id]) > 0:

            ops = self.meas[ps.header.frame_id][-1][1]

            if (np.dot(q2a(ops.pose.orientation), q2a(ps.pose.orientation))) < 0.0:
                ps.pose.orientation.x *= -1.0
                ps.pose.orientation.y *= -1.0
                ps.pose.orientation.z *= -1.0
                ps.pose.orientation.w *= -1.0

        try:

            self.meas[ps.header.frame_id].append([dist, self.transform(ps)])

        except tf.Exception:

            rospy.logwarn("Transform between " + self.target_frame +
                          " and " + ps.header.frame_id + " not available!")

    def prune_meas(self, now):

        frames_to_delete = []

        # delete old measurements
        for frame_id, val in self.meas.iteritems():

            idx_to_delete = []

            for idx in range(0, len(val)):

                if (now - val[idx][1].header.stamp) > rospy.Duration(1.0):
                    idx_to_delete.append(idx)

            val = [x for i, x in enumerate(val) if i not in idx_to_delete]

            if len(val) == 0:
                frames_to_delete.append(frame_id)

            self.meas[frame_id] = val

        for frame_id in frames_to_delete:
            del self.meas[frame_id]

    def get_inst(self):

        w = []

        px = []
        py = []
        pz = []
        ox = []
        oy = []
        oz = []
        ow = []

        for frame_id, val in self.meas.iteritems():

            dist = []

            for v in val:
                dist.append(v[0])

            adist = np.mean(dist)

            for v in val:
                w.append(1.0 / (adist ** 2))

                px.append(v[1].pose.position.x)
                py.append(v[1].pose.position.y)
                pz.append(v[1].pose.position.z)

                ox.append(v[1].pose.orientation.x)
                oy.append(v[1].pose.orientation.y)
                oz.append(v[1].pose.orientation.z)
                ow.append(v[1].pose.orientation.w)

        if len(w) < self.min_meas_cnt:
            return None

        self.inst.pose.position.x = np.average(px, weights=w)
        self.inst.pose.position.y = np.average(py, weights=w)
        self.inst.pose.position.z = np.average(pz, weights=w)

        # here we assume that quaternions are close (object is static) ->
        # averaging should be fine
        self.inst.pose.orientation.x = np.average(ox, weights=w)
        self.inst.pose.orientation.y = np.average(oy, weights=w)
        self.inst.pose.orientation.z = np.average(oz, weights=w)
        self.inst.pose.orientation.w = np.average(ow, weights=w)

        self.inst.pose.orientation = normalize(self.inst.pose.orientation)

        return self.inst

    def transform(self, ps):

        self.tfl.waitForTransform(
            self.target_frame, ps.header.frame_id, ps.header.stamp, rospy.Duration(0.5))

        return self.tfl.transformPose(self.target_frame, ps)


# "tracking" of static objects
class ArtSimpleTracker:
    def __init__(self, target_frame="/marker"):

        self.target_frame = target_frame
        self.tfl = tf.TransformListener()
        self.lock = threading.Lock()
        self.sub = rospy.Subscriber(
            "/art/object_detector/object", InstancesArray, self.cb, queue_size=10)
        self.pub = rospy.Publisher(
            "/art/object_detector/object_filtered", InstancesArray, queue_size=10, latch=True)
        self.timer = rospy.Timer(rospy.Duration(0.5), self.timer_cb)
        self.objects = {}

    def timer_cb(self, event):

        with self.lock:

            ia = InstancesArray()
            ia.header.frame_id = self.target_frame
            ia.header.stamp = rospy.Time.now()

            objects_to_delete = []

            for k, v in self.objects.iteritems():

                v.prune_meas(ia.header.stamp)

                inst = v.get_inst()

                if inst is None:  # new object might not have enough measurements yet

                    # TODO fix it: this would keep objects which were detected only few times
                    if not v.new:  # object is no longer detected

                        objects_to_delete.append(k)
                        ia.lost_objects.append(k)
                        continue

                    continue

                if v.new:
                    v.new = False
                    ia.new_objects.append(k)

                ia.instances.append(inst)

            for obj_id in objects_to_delete:
                del self.objects[obj_id]

            self.pub.publish(ia)

    def cb(self, msg):

        with self.lock:

            if msg.header.frame_id == self.target_frame:
                rospy.logwarn_throttle(1.0, "Some detections are already in target frame!")
                return

            for inst in msg.instances:

                if inst.object_id in self.objects:

                    rospy.logdebug("Updating object: " + inst.object_id)

                else:

                    rospy.loginfo("Adding new object: " + inst.object_id)
                    self.objects[inst.object_id] = TrackedObject(self.target_frame, self.tfl, inst)

                ps = PoseStamped()
                ps.header = msg.header
                ps.pose = inst.pose
                self.objects[inst.object_id].add_meas(ps)


if __name__ == '__main__':
    try:
        rospy.init_node('simple_tracker')
        ArtSimpleTracker()
        rospy.spin()
    except rospy.ROSInterruptException:
        print "program interrupted before completion"
