#!/usr/bin/env python

import rospy
from std_msgs.msg import Bool
from std_srvs.srv import Trigger, TriggerResponse
from art_msgs.msg import ObjInstance


class FakeVisualInspection:

    def __init__(self):
        self.vi_result = rospy.Publisher('/art/visual_inspection/result', Bool, queue_size=1)
        self.vi_srv = rospy.Service('/art/visual_inspection/start', Trigger, self.vi_srv_cb)
        self.grasped_object_sub = rospy.Subscriber(
            "/art/robot/right_arm/grasped_object", ObjInstance, self.grasped_obj_cb)
        self.grasped_obj = None

    def timer_cb(self, evt):
        if self.grasped_obj and self.grasped_obj in ["2002", "2008", "1011", "1024"]:
            res = False
        else:
            res = True
        self.vi_result.publish(res)

    def vi_srv_cb(self, request):
        rospy.Timer(rospy.Duration(1), self.timer_cb, oneshot=True)
        return TriggerResponse(success=True)

    def grasped_obj_cb(self, obj):
        self.grasped_obj = obj.object_id


if __name__ == '__main__':
    rospy.init_node('fake_visual_inspection')
    ''',log_level=rospy.DEBUG'''

    try:
        node = FakeVisualInspection()

        rospy.spin()
    except rospy.ROSInterruptException:
        pass
