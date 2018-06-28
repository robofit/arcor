#!/usr/bin/env python

import rospy
from std_srvs.srv import Trigger, TriggerResponse


class FakeUserState:

    def __init__(self):

        self.left_get_ready = rospy.Service("/art/dobot/get_ready", Trigger,
                                            self.get_ready_cb)

        rospy.loginfo("Server ready")

    def get_ready_cb(self, req):

        resp = TriggerResponse()
        resp.success = True
        return resp


if __name__ == '__main__':
    rospy.init_node('fake_basic_control')
    ''',log_level=rospy.DEBUG'''

    try:
        node = FakeUserState()

        rospy.spin()
    except rospy.ROSInterruptException:
        pass
