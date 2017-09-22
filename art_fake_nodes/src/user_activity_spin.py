#!/usr/bin/env python

import rospy
from actionlib import SimpleActionServer
from art_msgs.msg import UserActivity
import time
import random


class FakeUserState:

    act = UserActivity()

    def __init__(self):
        self.activity_publisher = rospy.Publisher('/art/user/activity', UserActivity, queue_size=10, latch=True)
        self.act.activity = UserActivity.AWAY
        self.activity_publisher.publish(self.act)
        rospy.loginfo("AWAY")

    def change_activity(self):
        if self.act == UserActivity.AWAY:
            self.act = UserActivity.READY
            rospy.loginfo("READY")
        elif self.act == UserActivity.READY:
            self.act = UserActivity.WORKING
            rospy.loginfo("WORKING")
        else:
            self.act = UserActivity.AWAY
            rospy.loginfo("AWAY")
        self.activity_publisher.publish(self.act)


if __name__ == '__main__':
    rospy.init_node('fake_user_activity_spin')
    ''',log_level=rospy.DEBUG'''

    try:
        node = FakeUserState()
        rate = rospy.Rate(1)
        while not rospy.is_shutdown():

            node.change_activity()

            rate.sleep()
    except rospy.ROSInterruptException:
        pass
