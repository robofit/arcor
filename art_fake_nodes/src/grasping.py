#!/usr/bin/env python

import rospy
from actionlib import SimpleActionServer
from art_msgs.msg import PickPlaceAction, PickPlaceGoal, PickPlaceResult, PickPlaceFeedback
import time
import random


class FakeGrasping:
    ALWAYS = 0
    NEVER = 1
    RANDOM = 2

    def __init__(self):
        self.server = SimpleActionServer('/art/pr2/left_arm/pp', PickPlaceAction,
                                         execute_cb=self.pick_place_left_cb)

        self.server = SimpleActionServer('/art/pr2/right_arm/pp', PickPlaceAction,
                                         execute_cb=self.pick_place_right_cb)
        self.objects = self.ALWAYS
        self.grasp = self.ALWAYS
        self.place = self.ALWAYS
        self.object_randomness = 0.8  # 80% of time object is known
        self.grasp_randomness = 0.4
        self.place_randomness = 0.4
        self.holding_left = False
        self.holding_right = False
        self.pick_length = 1  # how long (sec) takes to pick an object
        self.place_length = 1  # how long (sec) takes to place an object

        random.seed()

    def pick_place_left_cb(self, goal):
        self.pickplace_cb(goal, True)

    def pick_place_right_cb(self, goal):
        self.pickplace_cb(goal, False)

    def pickplace_cb(self, goal, left=True):
        result = PickPlaceResult()
        feedback = PickPlaceFeedback()
        if not (goal.operation == PickPlaceGoal.PICK_OBJECT_ID or
                goal.operation == PickPlaceGoal.PICK_FROM_FEEDER or
                goal.operation == PickPlaceGoal.PLACE_TO_POSE):
            result.result = PickPlaceResult.BAD_REQUEST
            rospy.logerr("BAD_REQUEST, Unknown operation")
            self.server.set_aborted(result, "Unknown operation")
            return

        if self.objects == self.ALWAYS:
            pass
        elif self.objects == self.NEVER:
            result.result = PickPlaceResult.BAD_REQUEST
            rospy.logerr("BAD_REQUEST, Unknown object id")
            self.server.set_aborted(result, "Unknown object id")
            return
        elif self.objects == self.RANDOM:
            nmbr = random.random()
            if nmbr > self.object_randomness:
                result.result = PickPlaceResult.BAD_REQUEST
                rospy.logerr("BAD_REQUEST, Unknown object id")
                self.server.set_aborted(result, "Unknown object id")
                return

        grasped = False

        if goal.operation == PickPlaceGoal.PICK_OBJECT_ID or goal.operation == PickPlaceGoal.PICK_FROM_FEEDER:
            rospy.sleep(self.pick_length)
            if (left and self.holding_left) or (not left and self.holding_right):
                result.result = PickPlaceResult.FAILURE
                rospy.logerr("Failure, already holding object in " +
                             "left" if left else "right" + "arm")
                self.server.set_aborted(
                    result, "Already holding object in " + "left" if left else "right" + "arm")
                return
            if self.grasp == self.ALWAYS:
                grasped = True
                pass
            elif self.grasp == self.NEVER:
                result.result = PickPlaceResult.FAILURE
                rospy.logerr("FAILURE, Pick Failed")
                self.server.set_aborted(result, "Pick Failed")
                return

            tries = 5
            max_attempts = 5
            while tries > 0:
                feedback.attempt = (max_attempts - tries) + 1
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
                result.result = PickPlaceResult.FAILURE
                self.server.set_aborted(result, "Pick failed")
                rospy.logerr("FAILURE, Pick Failed")
                return
            else:
                if left:
                    self.holding_left = True
                else:
                    self.holding_right = True
        placed = False
        if goal.operation == PickPlaceGoal.PLACE_TO_POSE:
            rospy.sleep(self.place_length)
            if (left and not self.holding_left) or (not left and not self.holding_right):
                result.result = PickPlaceResult.FAILURE
                rospy.logerr("Failure, already holding object in " +
                             "left" if left else "right" + "arm")
                self.server.set_aborted(
                    result, "Already holding object in " + "left" if left else "right" + "arm")
                return
            if self.place == self.ALWAYS:
                placed = True
                pass
            elif self.place == self.NEVER:
                result.result = PickPlaceResult.FAILURE
                self.server.set_aborted(result, "Place Failed")
                rospy.logerr("FAILURE, Place Failed")
                return

            tries = 5
            max_attempts = 5
            while tries > 0:
                feedback.attempt = (max_attempts - tries) + 1
                tries -= 1
                self.server.publish_feedback(feedback)

                if self.place == self.RANDOM:
                    nmbr = random.random()
                    if nmbr < self.place_randomness:
                        placed = True
                if placed:
                    break
            if not placed:
                result.result = PickPlaceResult.FAILURE
                self.server.set_aborted(result, "Place failed")
                rospy.logerr("FAILURE, Place Failed")
                return
            else:
                if left:
                    self.holding_left = False
                else:
                    self.holding_right = False

        result.result = PickPlaceResult.SUCCESS
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
