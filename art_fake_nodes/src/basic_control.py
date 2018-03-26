#!/usr/bin/env python

import rospy
from std_srvs.srv import Empty, EmptyResponse, Trigger, TriggerResponse
from std_msgs.msg import Bool, Float32
from geometry_msgs.msg import PointStamped
from trajectory_msgs.msg import JointTrajectory


class FakeUserState:

    def __init__(self):
        self.left_arm_mann = False
        self.right_arm_mann = False

        self.left_interaction_on = rospy.Service("/art/robot/left_arm/interaction/on", Trigger, self.left_interaction_on_cb)
        self.left_interaction_off = rospy.Service("/art/robot/left_arm/interaction/off", Trigger, self.left_interaction_off_cb)
        self.left_get_ready = rospy.Service("/art/robot/left_arm/get_ready", Trigger,
                                            self.left_interaction_get_ready_cb)
        self.left_move_to_user = rospy.Service("/art/robot/left_arm/move_to_user", Trigger,
                                               self.left_interaction_move_to_user_cb)
        self.left_arm_up = rospy.Service("/art/robot/left_arm/arm_up", Trigger,
                                         self.arm_up_cb)
        self.left_int_pub = rospy.Publisher("/art/robot/left_arm/interaction/state", Bool, queue_size=1, latch=True)

        self.right_interaction_on = rospy.Service("/art/robot/right_arm/interaction/on", Trigger, self.right_interaction_on_cb)
        self.right_interaction_off = rospy.Service("/art/robot/right_arm/interaction/off", Trigger, self.right_interaction_off_cb)
        self.right_get_ready = rospy.Service("/art/robot/right_arm/get_ready", Trigger,
                                             self.right_interaction_get_ready_cb)
        self.right_move_to_user = rospy.Service("/art/robot/right_arm/move_to_user", Trigger,
                                                self.right_interaction_move_to_user_cb)
        self.right_arm_up = rospy.Service("/art/robot/right_arm/arm_up", Trigger,
                                          self.arm_up_cb)
        self.right_int_pub = rospy.Publisher("/art/robot/right_arm/interaction/state", Bool, queue_size=1, latch=True)

        rospy.loginfo("Server ready")
        self.spine_up_service = rospy.Service("/art/robot/spine/up", Empty, self.spine_up_cb)
        self.spine_down_service = rospy.Service("/art/robot/spine/down", Empty, self.spine_down_cb)
        self.spine_control_sub = rospy.Subscriber("/art/robot/spine/control", Float32, self.spine_control_cb)
        self.look_at_sub = rospy.Subscriber("/art/robot/look_at", PointStamped, self.look_at_cb)
        self.spine_control_pub = rospy.Publisher("/art/pr2/torso_controller/command", JointTrajectory, queue_size=1)

        self.left_int_pub.publish(False)
        self.right_int_pub.publish(False)

    def arm_up_cb(self, req):

        resp = TriggerResponse()
        resp.success = True
        return resp

    def left_interaction_on_cb(self, req):
        resp = TriggerResponse()
        if self.left_arm_mann:
            rospy.logerr('Left arm already in interactive mode')
            resp.success = False
        else:
            rospy.loginfo('Left arm interactive mode ON')
            self.left_arm_mann = True
            resp.success = True
            self.left_int_pub.publish(True)
        return resp

    def left_interaction_off_cb(self, req):
        resp = TriggerResponse()
        if not self.left_arm_mann:
            rospy.logerr('Left arm already in normal mode')
            resp.success = False
        else:
            rospy.loginfo('Left arm interactive mode OFF')
            self.left_arm_mann = False
            resp.success = True
            self.left_int_pub.publish(False)
        return resp

    def left_interaction_get_ready_cb(self, req):

        if self.left_arm_mann:
            rospy.logerr('Left arm in interactive mode')
        else:
            rospy.loginfo('Left arm in GET READY position')
            pass
        resp = TriggerResponse()
        resp.success = True
        return resp

    def left_interaction_move_to_user_cb(self, req):

        if self.left_arm_mann:
            rospy.logerr('Left arm in interactive mode')
        else:
            rospy.loginfo('Left arm moved TO USER')
        resp = TriggerResponse()
        resp.success = True
        return resp

    def right_interaction_on_cb(self, req):
        resp = TriggerResponse()

        if self.right_arm_mann:
            rospy.logerr('Right arm already in interactive mode')
            resp.success = False
        else:
            rospy.loginfo('Right arm interactive mode ON')
            resp.success = True
            self.right_arm_mann = True
            self.left_int_pub.publish(True)

        return resp

    def right_interaction_off_cb(self, req):
        resp = TriggerResponse()

        if not self.right_arm_mann:
            rospy.logerr('Right arm already in normal mode')
            resp.success = False
        else:
            rospy.loginfo('Right arm interactive mode OFF')
            resp.success = True
            self.right_arm_mann = False
            self.left_int_pub.publish(False)

        return resp

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
