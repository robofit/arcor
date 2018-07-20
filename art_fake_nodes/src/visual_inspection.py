#!/usr/bin/env python

import rospy
from std_msgs.msg import Bool
from std_srvs.srv import Trigger, TriggerResponse
import random


class FakeVisualInspection:

    l = (True, True, False)

    def __init__(self):
        self.vi_result = rospy.Publisher('/art/visual_inspection/result', Bool, queue_size=1)
        self.vi_srv = rospy.Service('/art/visual_inspection/start', Trigger, self.vi_srv_cb)

    def timer_cb(self, evt):
        self.vi_result.publish(Bool(data=random.choice(self.l)))

    def vi_srv_cb(self, request):
        rospy.Timer(rospy.Duration(0.1), self.timer_cb, oneshot=True)
        return TriggerResponse(success=True)


if __name__ == '__main__':
    rospy.init_node('fake_visual_inspection')
    ''',log_level=rospy.DEBUG'''

    try:
        node = FakeVisualInspection()

        rospy.spin()
    except rospy.ROSInterruptException:
        pass
