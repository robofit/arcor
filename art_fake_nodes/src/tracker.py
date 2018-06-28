#!/usr/bin/env python

import rospy
from art_msgs.msg import ObjInstance, InstancesArray
from geometry_msgs.msg import Pose
from std_msgs.msg import Header
from std_srvs.srv import Empty, EmptyResponse
from art_msgs.srv import ObjectFlagClear, ObjectFlagSet, ObjectFlagSetResponse, ObjectFlagClearResponse


class FakeObjectTracker:

    def __init__(self):
        self.object_publisher = rospy.Publisher('/art/object_detector/object_filtered',
                                                InstancesArray, queue_size=10, latch=True)

        self.srv_set_flag = rospy.Service('/art/object_detector/flag/set', ObjectFlagSet, self.set_cb)
        self.srv_clear_flag = rospy.Service('/art/object_detector/flag/clear', ObjectFlagClear, self.clear_cb)
        self.srv_clear_all_flags = rospy.Service('/art/object_detector/flag/clear_all',
                                                 Empty, self.empty_cb)

        self.forearm_cams = ("/l_forearm_cam_optical_frame", "/r_forearm_cam_optical_frame")
        self.srv_enable_forearm = rospy.Service('/art/object_detector/forearm/enable',
                                                Empty, self.empty_cb)
        self.srv_disable_forearm = rospy.Service('/art/object_detector/forearm/disable',
                                                 Empty, self.empty_cb)
        self.srv_enable_detection = rospy.Service('/art/object_detector/all/enable', Empty,
                                                  self.empty_cb)
        self.srv_disable_detection = rospy.Service('/art/object_detector/all/disable', Empty,
                                                   self.empty_cb)

        self.objects = []
        obj = ObjInstance()
        obj.object_id = "profile_20_60"
        obj.object_type = "profile_20_60"

        obj.pose = Pose()
        obj.pose.position.x = 0.3
        obj.pose.position.y = 0.3
        obj.pose.position.z = 0
        obj.pose.orientation.x = 0
        obj.pose.orientation.y = 0
        obj.pose.orientation.z = 0
        obj.pose.orientation.w = 1
        self.objects.append(obj)
        obj = ObjInstance()
        obj.object_id = "profile_21_60"
        obj.object_type = "profile_20_60"

        obj.pose = Pose()
        obj.pose.position.x = 0.75
        obj.pose.position.y = 0.58
        obj.pose.position.z = 0
        obj.pose.orientation.x = 0
        obj.pose.orientation.y = 0
        obj.pose.orientation.z = 0
        obj.pose.orientation.w = 1
        self.objects.append(obj)

    def publish_objects(self):

        objs = InstancesArray()
        for obj in self.objects:
            objs.instances.append(obj)
        objs.header = Header()
        objs.header.frame_id = "marker"
        self.object_publisher.publish(objs)

    def set_cb(self, request):
        r = ObjectFlagSetResponse()
        r.success = True
        return r

    def clear_cb(self, request):
        r = ObjectFlagClearResponse()
        r.success = True
        return r

    def empty_cb(self, request):
        return EmptyResponse()


if __name__ == '__main__':
    rospy.init_node('fake_tracker')
    ''',log_level=rospy.DEBUG'''

    try:
        node = FakeObjectTracker()
        rate = rospy.Rate(1)
        while not rospy.is_shutdown():
            node.publish_objects()
            rate.sleep()
    except rospy.ROSInterruptException:
        pass
