#! /usr/bin/env python

import roslib; roslib.load_manifest('actionlib_tutorials')
import rospy

import actionlib
import art_umf_localizer.msg

if __name__ == '__main__':

  rospy.init_node('umf_localizer_client_py')
  client = actionlib.SimpleActionClient('/umf_localizer_node/localize', art_umf_localizer.msg.LocalizeAgainstUMFAction)
  client.wait_for_server()
  goal = art_umf_localizer.msg.LocalizeAgainstUMFGoal()
  goal.timeout = rospy.Duration(10)
  client.send_goal(goal)
  client.wait_for_result()
  print(client.get_result())
