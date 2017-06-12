#!/usr/bin/env python

import rospy
from actionlib import SimpleActionServer
from art_msgs.msg import ArmNavigationAction, ArmNavigationResult


class FakeArmNavigation():

    def __init__(self, name):

        _as = SimpleActionServer(name, ArmNavigationAction, self.action_cb)

    def action_cb(self, req):

        result = ArmNavigationResult()
        result.result = ArmNavigationResult.SUCCESS

        self._as.set_succeeded(result)


def main():

    FakeArmNavigation("/art/pr2/left_arm/manipulation")
    FakeArmNavigation("/art/pr2/right_arm/manipulation")
    rospy.spin()


if __name__ == '__main__':
    rospy.init_node('fake_arm_navigation')
    ''',log_level=rospy.DEBUG'''

    try:
        main()
    except rospy.ROSInterruptException:
        pass
