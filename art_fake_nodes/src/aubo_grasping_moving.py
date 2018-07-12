#!/usr/bin/env python

import rospy
from actionlib import SimpleActionServer
import random
from kinali_msgs.msg import RobotMoveAction, RobotMoveGoal, RobotMoveResult, RobotMoveFeedback, RobotStatus
from geometry_msgs.msg import PoseStamped, Pose
from std_msgs.msg import Header


class FakeGrasping:
    ALWAYS = 0
    NEVER = 1
    RANDOM = 2

    def __init__(self):
        self.server = SimpleActionServer('/robot_move', RobotMoveAction,
                                         execute_cb=self.robot_move_cb)

        self.objects = self.ALWAYS
        self.grasp = self.ALWAYS
        self.place = self.ALWAYS
        self.object_randomness = 0.8  # 80% of time object is known
        self.grasp_randomness = 0.4
        self.place_randomness = 0.4
        self.holding = None
        self.pick_length = 2  # how long (sec) takes to pick an object
        self.place_length = 2  # how long (sec) takes to place an object

        self.robot_state_pub = rospy.Publisher("/robot_status", RobotStatus, queue_size=1, latch=True)
        self.publish_robot_status(RobotStatus.STOPPED)

        random.seed()

    def publish_robot_status(self, status):
        self.robot_state_pub.publish(RobotStatus(status=status,
                                                 actual_pose=PoseStamped(header=Header(frame_id="marker"))))

    def robot_move_cb(self, goal):
        self.publish_robot_status(RobotStatus.MOVING)
        self.robot_move(goal)
        self.publish_robot_status(RobotStatus.STOPPED)

    def robot_move(self, goal):
        result = RobotMoveResult()
        feedback = RobotMoveFeedback()
        if not (goal.move_type in (goal.HOME, goal.PROGRAMING, goal.PICK, goal.PLACE, goal.MOVE)):
            result.result = RobotMoveResult.BAD_REQUEST
            rospy.logerr("BAD_REQUEST, Unknown operation")
            self.server.set_aborted(result, "Unknown operation")

            return

        if self.objects == self.ALWAYS:
            pass
        elif self.objects == self.NEVER:
            result.result = RobotMoveResult.BAD_REQUEST
            rospy.logerr("BAD_REQUEST, Unknown object id")
            self.server.set_aborted(result, "Unknown object id")
            return
        elif self.objects == self.RANDOM:
            nmbr = random.random()
            if nmbr > self.object_randomness:
                result.result = RobotMoveResult.BAD_REQUEST
                rospy.logerr("BAD_REQUEST, Unknown object id")
                self.server.set_aborted(result, "Unknown object id")
                return

        grasped = False

        if goal.move_type == RobotMoveGoal.PICK:
            rospy.sleep(self.pick_length)
            if False and self.holding:
                result.result = RobotMoveResult.BUSY
                rospy.logerr("Failure, already holding object in arm")
                self.server.set_aborted(
                    result, "Already holding object in arm")
                return
            if self.grasp == self.ALWAYS:
                grasped = True
                pass
            elif self.grasp == self.NEVER:
                result.result = RobotMoveResult.MOVE_FAILURE
                rospy.logerr("FAILURE, Pick Failed")
                self.server.set_aborted(result, "Pick Failed")
                return

            tries = 5
            while tries > 0:
                feedback.state = feedback.PICKING
                tries -= 1
                self.server.publish_feedback(feedback)

                if self.grasp == self.RANDOM:
                    nmbr = random.random()
                    if nmbr < self.grasp_randomness:
                        grasped = True
                if grasped:
                    break

            if self.server.is_preempt_requested():
                self.server.set_preempted(result, "Pick canceled")
                rospy.logerr("Preempted")
                return

            if not grasped:
                result.result = RobotMoveResult.FAILURE
                self.server.set_aborted(result, "Pick failed")
                rospy.logerr("FAILURE, Pick Failed")
                return
            else:
                self.holding = True

        placed = False
        if goal.move_type == RobotMoveGoal.PLACE:
            rospy.sleep(self.place_length)
            if not self.holding:
                result.result = RobotMoveResult.MOVE_FAILURE
                rospy.logerr("Failure, robot not holding object")
                self.server.set_aborted(
                    result, "Robot not holding object")
                return
            if self.place == self.ALWAYS:
                placed = True
                pass
            elif self.place == self.NEVER:
                result.result = RobotMoveResult.MOVE_FAILURE
                self.server.set_aborted(result, "Place Failed")
                rospy.logerr("FAILURE, Place Failed")
                return

            tries = 5
            while tries > 0:
                feedback.state = feedback.PLACING
                tries -= 1
                self.server.publish_feedback(feedback)

                if self.place == self.RANDOM:
                    nmbr = random.random()
                    if nmbr < self.place_randomness:
                        placed = True
                if placed:
                    break
            if not placed:
                result.result = RobotMoveResult.MOVE_FAILURE
                self.server.set_aborted(result, "Place failed")
                rospy.logerr("FAILURE, Place Failed")
                return
            else:
                self.holding = False

        result.result = RobotMoveResult.SUCCES
        self.server.set_succeeded(result)
        rospy.loginfo("SUCCESS")
        print("Finished")


if __name__ == '__main__':
    rospy.init_node('fake_grasping')
    ''',log_level=rospy.DEBUG'''

    try:
        node = FakeGrasping()

        rospy.spin()
    except rospy.ROSInterruptException:
        pass
