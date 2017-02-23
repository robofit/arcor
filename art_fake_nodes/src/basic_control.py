#!/usr/bin/env python

import rospy
from actionlib import SimpleActionServer
from art_msgs.msg import UserStatus
import time
import random


class FakeUserState:

    def __init__(self):
        state_publisher = rospy.Publisher('/art/user/status', UserStatus, queue_size=10, latch=True)
        st = UserStatus()
        st.user_id = 1
        st.user_state = UserStatus.USER_NOT_CALIBRATED
        state_publisher.publish(st)
        time.sleep(2)
        st.user_state = UserStatus.USER_CALIBRATED
        state_publisher.publish(st)


if __name__ == '__main__':
    rospy.init_node('fake_user_state')
    ''',log_level=rospy.DEBUG'''

    try:
        node = FakeUserState()

        rospy.spin()
    except rospy.ROSInterruptException:
        pass
