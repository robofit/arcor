#!/usr/bin/env python

import rospy
from actionlib import SimpleActionServer
from art_msgs.msg import UserStatus
import time
import random
from std_srvs.srv import Empty, EmptyResponse, Trigger, TriggerResponse
from std_msgs.msg import Bool, Float32
from geometry_msgs.msg import PointStamped
from trajectory_msgs.msg import JointTrajectory


class FakeUserState:

    def __init__(self):

        self.left_get_ready = rospy.Service("/art/dobot/get_ready", Trigger,
                                            self.get_ready_cb)

        rospy.loginfo("Server ready")

    def get_ready_cb(self, req):

        resp = TriggerResponse()
        resp.success = True
        return resp


if __name__ == '__main__':
    rospy.init_node('fake_basic_control')
    ''',log_level=rospy.DEBUG'''

    try:
        node = FakeUserState()

        rospy.spin()
    except rospy.ROSInterruptException:
        pass
