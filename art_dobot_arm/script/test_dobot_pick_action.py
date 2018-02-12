#! /usr/bin/env python

import rospy
import actionlib
from art_msgs.msg import PickPlaceAction, PickPlaceGoal, PickPlaceResult
from geometry_msgs.msg import PoseStamped


def pick_object_id(client, object_id):

    rospy.loginfo('pick_object_id: ' + str(object_id))
    goal = PickPlaceGoal()
    goal.object = object_id
    goal.operation = goal.PICK_OBJECT_ID
    return send_goal(client, goal)


def get_ready(client):

    rospy.loginfo('get_ready')
    goal = PickPlaceGoal()
    goal.operation = goal.GET_READY
    return send_goal(client, goal)


def place_object(client, x, y):

    rospy.loginfo('place_object: ' + str((x, y)))
    goal = PickPlaceGoal()
    goal.operation = goal.PLACE_TO_POSE
    goal.z_axis_angle_increment = 1.57
    goal.keep_orientation = False

    goal.pose = PoseStamped()
    goal.pose.header.frame_id = "marker"
    goal.pose.header.stamp = rospy.Time.now()
    goal.pose.pose.position.x = x
    goal.pose.pose.position.y = y
    goal.pose.pose.position.z = -0.03
    goal.pose.pose.orientation.w = 1.0

    return send_goal(client, goal)


def send_goal(client, goal):

    rospy.loginfo('sending goal')
    client.send_goal(goal)

    client.wait_for_result()

    rospy.loginfo('got result')
    print client.get_result()
    print "status: " + client.get_goal_status_text()
    print "state: " + str(client.get_state())
    print

    return client.get_result().result


def main():

    arm_client = actionlib.SimpleActionClient('/art/dobot/pp_client', PickPlaceAction)
    arm_client.wait_for_server()

    if get_ready(arm_client) != PickPlaceResult.SUCCESS:
        return

    if pick_object_id(arm_client, "obj") != PickPlaceResult.SUCCESS:
        return

    if get_ready(arm_client) != PickPlaceResult.SUCCESS:
        return

    if place_object(arm_client, -0.025, -0.288) != PickPlaceResult.SUCCESS:
        return

    if get_ready(arm_client) != PickPlaceResult.SUCCESS:
        return

    rospy.loginfo("Done!")


if __name__ == '__main__':
    try:
        rospy.init_node('pp_client_py')
        main()
    except rospy.ROSInterruptException:
        print "program interrupted before completion"
