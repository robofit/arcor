#!/usr/bin/env python

import rospy
from art_msgs.msg import UserActivity


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
    rospy.init_node('fake_user_activity')
    ''',log_level=rospy.DEBUG'''

    try:
        node = FakeUserState()
        rate = rospy.Rate(30)
        while not rospy.is_shutdown():
            try:
                raw_input("Press Enter to change activity type")
                node.change_activity()
            except KeyboardInterrupt:
                pass
            rate.sleep()
    except rospy.ROSInterruptException:
        pass
