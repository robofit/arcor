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
        self.left_arm_mann = False
        self.right_arm_mann = False

        self.left_interaction_on = rospy.Service("/art/pr2/left_arm/interaction/on", Empty, self.left_interaction_on_cb)
        self.left_interaction_off = rospy.Service("/art/pr2/left_arm/interaction/off", Empty, self.left_interaction_off_cb)
        self.left_get_ready = rospy.Service("/art/pr2/left_arm/get_ready", Trigger,
                                            self.left_interaction_get_ready_cb)
        self.left_move_to_user = rospy.Service("/art/pr2/left_arm/move_to_user", Trigger,
                                               self.left_interaction_move_to_user_cb)
        self.left_int_pub = rospy.Publisher("/art/pr2/left_arm/interaction/state", Bool, queue_size=1, latch=True)

        self.right_interaction_on = rospy.Service("/art/pr2/right_arm/interaction/on", Empty, self.right_interaction_on_cb)
        self.right_interaction_off = rospy.Service("/art/pr2/right_arm/interaction/off", Empty, self.right_interaction_off_cb)
        self.right_get_ready = rospy.Service("/art/pr2/right_arm/get_ready", Trigger,
                                             self.right_interaction_get_ready_cb)
        self.right_move_to_user = rospy.Service("/art/pr2/right_arm/move_to_user", Trigger,
                                                self.right_interaction_move_to_user_cb)
        self.right_int_pub = rospy.Publisher("/art/pr2/right_arm/interaction/state", Bool, queue_size=1, latch=True)

        rospy.loginfo("Server ready")
        self.spine_up_service = rospy.Service("/art/pr2/spine/up", Empty, self.spine_up_cb)
        self.spine_down_service = rospy.Service("/art/pr2/spine/down", Empty, self.spine_down_cb)
        self.spine_control_sub = rospy.Subscriber("/art/pr2/spine/control", Float32, self.spine_control_cb)
        self.look_at_sub = rospy.Subscriber("/art/pr2/look_at", PointStamped, self.look_at_cb)
        self.spine_control_pub = rospy.Publisher("/art/pr2/torso_controller/command", JointTrajectory, queue_size=1)

    def left_interaction_on_cb(self,  req):

        if self.left_arm_mann:
            rospy.logerr('Left arm already in interactive mode')
        else:
            rospy.loginfo('Left arm interactive mode ON')
            self.left_arm_mann = True

        return EmptyResponse()

    def left_interaction_off_cb(self,  req):

        if not self.left_arm_mann:
            rospy.logerr('Left arm already in normal mode')
        else:
            rospy.loginfo('Left arm interactive mode OFF')
            self.left_arm_mann = False

        return EmptyResponse()

    def left_interaction_get_ready_cb(self,  req):

        if self.left_arm_mann:
            rospy.logerr('Left arm in interactive mode')
        else:
            rospy.loginfo('Left arm in GET READY position')
            pass
        resp = TriggerResponse()
        resp.success = True
        return resp

    def left_interaction_move_to_user_cb(self,  req):

        if self.left_arm_mann:
            rospy.logerr('Left arm in interactive mode')
        else:
            rospy.loginfo('Left arm moved TO USER')
        resp = TriggerResponse()
        resp.success = True
        return resp

    def right_interaction_on_cb(self,  req):

        if self.right_arm_mann:
            rospy.logerr('Right arm already in interactive mode')
        else:
            rospy.loginfo('Right arm interactive mode ON')
            self.right_arm_mann = True

        return EmptyResponse()

    def right_interaction_off_cb(self,  req):

        if not self.right_arm_mann:
            rospy.logerr('Right arm already in normal mode')
        else:
            rospy.loginfo('Right arm interactive mode OFF')
            self.right_arm_mann = False

        return EmptyResponse()

    def right_interaction_get_ready_cb(self, req):

        if self.right_arm_mann:
            rospy.logerr('Right arm in interactive mode')
        else:
            rospy.loginfo('Right arm in GET READY position')
        resp = TriggerResponse()
        resp.success = True
        return resp

    def right_interaction_move_to_user_cb(self, req):

        if self.right_arm_mann:
            rospy.logerr('Right arm in interactive mode')
        else:
            rospy.loginfo('Right arm moved TO USER')
        resp = TriggerResponse()
        resp.success = True
        return resp

    def spine_up_cb(self, empty):
        rospy.loginfo("Spine up")
        return EmptyResponse()

    def spine_down_cb(self, empty):
        rospy.loginfo("Spine down")
        return EmptyResponse()

    def spine_control_cb(self, height):
        """ height: Float32."""

        rospy.loginfo("Spine moved to desired position")

    def look_at_cb(self, where):
        """ where: PoseStamped."""
        rospy.loginfo("I looked where you wanted to")

if __name__ == '__main__':
    rospy.init_node('fake_basic_control')
    ''',log_level=rospy.DEBUG'''

    try:
        node = FakeUserState()

        rospy.spin()
    except rospy.ROSInterruptException:
        pass
