#!/usr/bin/env python
import roslib

from pr2_controllers_msgs.msg import JointTrajectoryAction, JointTrajectoryGoal
from trajectory_msgs.msg import JointTrajectoryPoint
import actionlib
import rospy
from copy import deepcopy
from sensor_msgs.msg import JointState


class Arm:
    def __init__(self, arm_name):
        # arm_name should be l_arm or r_arm
        self.name = arm_name
        self.jta = actionlib.SimpleActionClient('/' + arm_name + '_controller/joint_trajectory_action',
                                                JointTrajectoryAction)
        rospy.loginfo('Waiting for joint trajectory action')
        self.jta.wait_for_server()
        rospy.loginfo('Found joint trajectory action!')
        self.joint_state_sub = rospy.Subscriber("/joint_states", JointState, self.js_cb)
        self.angles = None
        char = self.name[0]
        self.joint_names = [char + '_shoulder_pan_joint',
                            char + '_shoulder_lift_joint',
                            char + '_upper_arm_roll_joint',
                            char + '_elbow_flex_joint',
                            char + '_forearm_roll_joint',
                            char + '_wrist_flex_joint',
                            char + '_wrist_roll_joint']

    def move(self):

        goal = JointTrajectoryGoal()
        goal.trajectory.joint_names = self.joint_names

        point = JointTrajectoryPoint()
        angles = self.angles
        for i in xrange(6 * 4):
            angles[6] += i * 0.1

            point.positions = angles
            point.time_from_start = rospy.Duration(1 * 0.25 * i)
            rospy.loginfo(str(angles[6]) + ": " + str(rospy.Duration(1 * 0.25 * i)))
            goal.trajectory.points.append(deepcopy(point))
        self.jta.send_goal_and_wait(goal)

    def js_cb(self, data):
        self.angles = [0] * 7
        positions = zip(data.name, data.position)
        for name, position in positions:
            if name in self.joint_names:
                self.angles[self.joint_names.index(name)] = position


def main():
    arm = Arm('r_arm')
    while arm.angles is None:
        rospy.sleep(1)
    arm.move()


if __name__ == '__main__':
    rospy.init_node('joint_position_tester')
    main()
