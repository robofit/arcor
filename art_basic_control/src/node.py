#!/usr/bin/env python

import rospy

import actionlib
from pr2_controllers_msgs.msg import PointHeadAction, PointHeadGoal
from std_srvs.srv import Empty, EmptyResponse
from std_msgs.msg import Float32, Bool
from geometry_msgs.msg import PointStamped, PoseStamped
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from pr2_mechanism_msgs.srv import SwitchController, SwitchControllerRequest
import tf


class ArtBasicControl:

    def __init__(self):
        self.head_action_client = actionlib.SimpleActionClient("/head_traj_controller/point_head_action", PointHeadAction)
        rospy.loginfo("Waiting for point_head_action server")
        self.head_action_client.wait_for_server()

        # taken from https://github.com/sniekum/pr2_lfd_utils/blob/master/src/pr2_lfd_utils/recordInteraction.py#L50
        rospy.loginfo('Waiting for pr2_controller_manager')
        rospy.wait_for_service('/pr2_controller_manager/switch_controller')
        self.switch_control = rospy.ServiceProxy('/pr2_controller_manager/switch_controller', SwitchController, persistent=True)
        self.standard_controllers = ['r_arm_controller', 'l_arm_controller']
        self.mannequin_controllers = ['r_arm_controller_loose', 'l_arm_controller_loose']
        self.left_arm_mann = False
        self.right_arm_mann = False
        self.switch_req = SwitchControllerRequest()
        self.switch_req.strictness = SwitchControllerRequest.BEST_EFFORT

        self.left_interaction_on = rospy.Service("left_arm/interaction/on", Empty, self.left_interaction_on_cb)
        self.left_interaction_off = rospy.Service("left_arm/interaction/off", Empty, self.left_interaction_off_cb)
        self.left_int_pub = rospy.Publisher("left_arm/interaction/state", Bool, queue_size=1, latch=True)

        self.right_interaction_on = rospy.Service("right_arm/interaction/on", Empty, self.right_interaction_on_cb)
        self.right_interaction_off = rospy.Service("right_arm/interaction/off", Empty, self.right_interaction_off_cb)
        self.right_int_pub = rospy.Publisher("right_arm/interaction/state", Bool, queue_size=1, latch=True)

        rospy.loginfo("Server ready")
        self.spine_up_service = rospy.Service("spine/up", Empty, self.spine_up_cb)
        self.spine_down_service = rospy.Service("spine/down", Empty, self.spine_down_cb)
        self.spine_control_sub = rospy.Subscriber("spine/control", Float32, self.spine_control_cb)
        self.look_at_sub = rospy.Subscriber("look_at", PointStamped, self.look_at_cb)
        self.spine_control_pub = rospy.Publisher("/torso_controller/command", JointTrajectory, queue_size=1)

        self.tfl = tf.TransformListener()
        self.left_gripper_pose_pub = rospy.Publisher("left_arm/gripper/pose", PoseStamped, queue_size=1)
        self.right_gripper_pose_pub = rospy.Publisher("right_arm/gripper/pose", PoseStamped, queue_size=1)
        self.gripper_pose_timer = rospy.Timer(rospy.Duration(0.2), self.gripper_pose_timer_cb)

    def publish_gripper_pose(self, frame_id, publisher):

        ps = PoseStamped()
        ps.header.frame_id = frame_id
        ps.header.stamp = rospy.Time(0)
        ps.pose.orientation.w = 1

        try:
            self.tfl.waitForTransform("/marker", ps.header.frame_id, ps.header.stamp, rospy.Duration(0.1))
            ps = self.tfl.transformPose("/marker", ps)
        except (tf.Exception, tf.ConnectivityException, tf.LookupException):
            rospy.logerr("Failed to transform gripper pose")
            return

        publisher.publish(ps)

    def gripper_pose_timer_cb(self, event):

        self.publish_gripper_pose("l_gripper_tool_frame",  self.left_gripper_pose_pub)
        self.publish_gripper_pose("r_gripper_tool_frame",  self.right_gripper_pose_pub)

    def left_interaction_on_cb(self,  req):

        if self.left_arm_mann:
            rospy.logerr('Left arm already in interactive mode')
        else:

            self.left_arm_mann = True

            self.switch_req.stop_controllers = [self.standard_controllers[1]]
            self.switch_req.start_controllers = [self.mannequin_controllers[1]]
            self.switch_control(self.switch_req)
            self.left_int_pub.publish(True)

        return EmptyResponse()

    def left_interaction_off_cb(self,  req):

        if not self.left_arm_mann:
            rospy.logerr('Left arm already in normal mode')
        else:

            self.left_arm_mann = False

            self.switch_req.stop_controllers = [self.mannequin_controllers[1]]
            self.switch_req.start_controllers = [self.standard_controllers[1]]
            self.switch_control(self.switch_req)
            self.left_int_pub.publish(False)

        return EmptyResponse()

    def right_interaction_on_cb(self,  req):

        if self.right_arm_mann:
            rospy.logerr('Right arm already in interactive mode')
        else:

            self.right_arm_mann = True

            self.switch_req.stop_controllers = [self.standard_controllers[0]]
            self.switch_req.start_controllers = [self.mannequin_controllers[0]]
            self.switch_control(self.switch_req)
            self.right_int_pub.publish(True)

        return EmptyResponse()

    def right_interaction_off_cb(self,  req):

        if not self.right_arm_mann:
            rospy.logerr('Right arm already in normal mode')
        else:

            self.right_arm_mann = False

            self.switch_req.stop_controllers = [self.mannequin_controllers[0]]
            self.switch_req.start_controllers = [self.standard_controllers[0]]
            self.switch_control(self.switch_req)
            self.right_int_pub.publish(False)

        return EmptyResponse()

    def spine_up_cb(self, empty):
        self.spine_move_to(1)
        return EmptyResponse()

    def spine_down_cb(self, empty):
        self.spine_move_to(0)
        return EmptyResponse()

    def spine_control_cb(self, height):
        """ height: Float32."""

        self.spine_move_to(height.data)

    def look_at_cb(self, where):
        """ where: PoseStamped."""

        goal = PointHeadGoal()
        goal.target = where
        goal.pointing_frame = "high_def_frame"
        goal.min_duration = rospy.Duration(0.5)
        goal.max_velocity = 0.5
        self.head_action_client.send_goal_and_wait(goal, rospy.Duration(5))

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
