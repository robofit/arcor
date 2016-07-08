#!/usr/bin/env python

import rospy
import time

import actionlib
from pr2_controllers_msgs.msg import PointHeadAction, PointHeadGoal
from std_srvs.srv import Empty, EmptyResponse
from std_msgs.msg import Float32
from geometry_msgs.msg import PointStamped
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint


class ArtBasicControl:

    def __init__(self):
        self.head_action_client = actionlib.SimpleActionClient("/head_traj_controller/point_head_action",
                                                                PointHeadAction)
        rospy.loginfo("Waiting for point_head_action server")
        self.head_action_client.wait_for_server()
        rospy.loginfo("Server ready")
        self.spine_up_service = rospy.Service("/art_basic_control/spine_up", Empty, self.spine_up_cb)
        self.spine_down_service = rospy.Service("/art_basic_control/spine_down", Empty, self.spine_down_cb)
        self.spine_control_sub = rospy.Subscriber("/art_basic_control/spine_control", Float32, self.spine_control_cb)
        self.look_at_sub = rospy.Subscriber("/art_basic_control/look_at", PointStamped, self.look_at_cb)
        self.spine_control_pub = rospy.Publisher("/torso_controller/command", JointTrajectory, queue_size=1)

    def spine_up_cb(self, empty):
        self.spine_move_to(1)
        return EmptyResponse()

    def spine_down_cb(self, empty):
        self.spine_move_to(0)
        return EmptyResponse()

    def spine_control_cb(self, height):
        """

        :type height: Float32
        :return:
        """
        self.spine_move_to(height.data)

    def look_at_cb(self, where):
        """

        :type where: PoseStamped
        :return:
        """
        goal = PointHeadGoal()
        goal.target = where
        goal.pointing_frame = "high_def_frame"
        goal.min_duration = rospy.Duration(0.5)
        goal.max_velocity = 0.8
        self.head_action_client.send_goal_and_wait(goal, rospy.Duration(2))

    def spine_move_to(self, height):

        if height < 0:
            height = 0
        elif height > 1.:
            height = 1.
        height_computed = height / (1 / 0.3)
        msg = JointTrajectory()
        msg.header.stamp = rospy.Time.now()
        msg.joint_names.append("torso_lift_joint")
        point = JointTrajectoryPoint()
        point.positions.append(height_computed)
        point.velocities.append(-0.01)
        msg.points.append(point)
        self.spine_control_pub.publish(msg)


if __name__ == '__main__':
    rospy.init_node('art_basic_control')
    try:
        node = ArtBasicControl()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
