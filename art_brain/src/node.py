#!/usr/bin/env python

import rospy
import time

import actionlib
from art_msgs.msg import LocalizeAgainstUMFAction, LocalizeAgainstUMFGoal, LocalizeAgainstUMFResult
from std_srvs.srv import Empty, EmptyRequest, EmptyResponse


class ArtBrain:

    def __init__(self):
        rospy.wait_for_service('/art_simple_gui/show_marker')
        try:
            show_marker = rospy.ServiceProxy('/art_simple_gui/show_marker', Empty)
            show_marker()

        except rospy.ServiceException, e:
            print "Service call failed: %s"%e
        print "started"
        table_calibration = True
        pr2_calibration = True

        while table_calibration:
            if self.calibrate_table():
                table_calibration = False
                rospy.loginfo("Table successfully calibrated")
            else:
                rospy.loginfo("Table calibration failed! Trying every 5 sec")
                time.sleep(5)
        while pr2_calibration:
            if self.calibrate_pr2():
                pr2_calibration = False
                rospy.loginfo("PR2 successfully calibrated")
            else:
                rospy.loginfo("PR2 calibration failed! Trying every 5 sec")
                time.sleep(5)

        rospy.wait_for_service('/art_simple_gui/hide_marker')
        try:
            hide_marker = rospy.ServiceProxy('/art_simple_gui/hide_marker', Empty)
            hide_marker()

        except rospy.ServiceException, e:
            print "Service call failed: %s"%e
        rospy.loginfo("Calibration done, hiding umf marker")
        pass

    def calibrate_table(self):
        client_table = actionlib.SimpleActionClient('/umf_localizer_node_table/localize', LocalizeAgainstUMFAction)
        rospy.loginfo("Waiting for table server")
        client_table.wait_for_server()
        rospy.loginfo("Table server ready")
        goal = LocalizeAgainstUMFGoal()
        goal.timeout = rospy.Duration(10)
        rospy.loginfo("Sending goal to table server")
        client_table.send_goal(goal)
        rospy.loginfo("Waiting for results (table)")
        client_table.wait_for_result()
        return not client_table.get_result().result

    def calibrate_pr2(self):
        client_pr2 = actionlib.SimpleActionClient('/umf_localizer_node/localize', LocalizeAgainstUMFAction)
        rospy.loginfo("Waiting for pr2 server")
        client_pr2.wait_for_server()
        rospy.loginfo("Pr2 server ready")
        goal = LocalizeAgainstUMFGoal()
        goal.timeout = rospy.Duration(10)
        rospy.loginfo("Sending goal to pr2 server")
        client_pr2.send_goal(goal)
        rospy.loginfo("Waiting for results (pr2)")
        client_pr2.wait_for_result()
        return not client_pr2.get_result().result

    def show_umf_marker(self):
        pass

    def hide_umf_marker(self):
        pass


if __name__ == '__main__':
    rospy.init_node('art_brain')
    try:
        node = ArtBrain()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
