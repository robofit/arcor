#!/usr/bin/env python

import rospy
import actionlib
from geometry_msgs.msg import PoseStamped, Pose
import copy
from moveit_msgs.msg import MoveGroupAction
from art_pr2_arm_navigation import ArtArmNavigationActionServer


class ArtPr2ArmNavigation(object):

    def __init__(self):

        rospy.loginfo('Waiting for move_group')
        moveit_action_client = actionlib.SimpleActionClient("/move_group", MoveGroupAction)
        moveit_action_client.wait_for_server()

        server_name_prefix = rospy.get_param("server_name", "/art/robot/")
        groups_names = rospy.get_param("groups", ['left_arm', 'right_arm'])
        groups = []
        for group in groups_names:
            groups.append(ArtArmNavigationActionServer(server_name_prefix, group))


if __name__ == '__main__':
    rospy.init_node('art_pr2_arm_navigation')
    try:
        node = ArtPr2ArmNavigation()
        rospy.spin()

    except rospy.ROSInterruptException:
        pass
