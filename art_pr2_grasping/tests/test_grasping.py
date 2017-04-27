#!/usr/bin/env python

import unittest
import rostest
import sys
import rospy
from std_msgs.msg import String
import actionlib
from art_msgs.msg import PickPlaceAction

class TestGrasping(unittest.TestCase):

    def setUp(self):

        self.grasped_object_sub = rospy.Subscriber('/art/pr2/left_arm/grasped_object', String, self.state_cb)

    def test_grasping(self):
        
        left_gr = rospy.wait_for_message('/art/pr2/left_arm/grasped_object', String)
        rgiht_gr = rospy.wait_for_message('/art/pr2/left_arm/grasped_object', String)
        
        self.assertEquals(left_gr.data, "", "test_left_arm_grasped_object")
        self.assertEquals(right_gr.data, "", "test_right_arm_grasped_object")
        
        left_client = actionlib.SimpleActionClient('/art/pr2/left_arm/pp',    art_msgs.msg.pickplaceAction)
        left_client.wait_for_server()
        
        right_client = actionlib.SimpleActionClient('/art/pr2/right_arm/pp',    art_msgs.msg.pickplaceAction)
        right_client.wait_for_server()
        
        # TODO simulate some object and try to grasp / place it
        

if __name__ == '__main__':

    rospy.init_node('test_grasping_node')
    rostest.run('art_pr2_grasping', 'test_grasping', TestGrasping, sys.argv)
