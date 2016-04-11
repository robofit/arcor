#!/usr/bin/env python

import rospy

import actionlib
from art_msgs.msg import LocalizeAgainstUMFAction, LocalizeAgainstUMFGoal, LocalizeAgainstUMFGoalState



class ArtBrain:

    def __init__(self):

        client_table = actionlib.SimpleActionClient('/umf_localizer_node_table/localize', LocalizeAgainstUMFAction)
        client_pr2 = actionlib.SimpleActionClient('/umf_localizer_node/localize', LocalizeAgainstUMFAction)
        client_table.wait_for_server()
        client_pr2.wait_for_server()
        goal = LocalizeAgainstUMFGoal()
        goal.timeout = rospy.Duration(10)
        client_table.send_goal(goal)
        client_pr2.send_goal(goal)
        client_table.wait_for_result()
        client_pr2.wait_for_result()

        #state = LocalizeAgainstUMFGoalState()
        state_pr2 = client_pr2.get_state()
        state_table = client_table.get_state()
        print state_pr2
        print state_table
        pass


if __name__ == '__main__':
    rospy.init_node('art_brain')
    try:
        node = ArtBrain()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
