#! /usr/bin/env python

import rospy
import actionlib
import art_msgs.msg
from art_msgs.msg import PickPlaceAction
from geometry_msgs.msg import PoseStamped


def pick_object_id(client,  object_id):

    goal = art_msgs.msg.PickPlaceGoal()
    goal.object = object_id
    goal.operation = goal.PICK_OBJECT_ID
    send_goal(client, goal)


def get_ready(client):

    goal = art_msgs.msg.PickPlaceGoal()
    goal.operation = goal.GET_READY
    send_goal(client, goal)


def place_object(client, x, y):

    goal = art_msgs.msg.PickPlaceGoal()
    goal.operation = goal.PLACE_TO_POSE

    goal.pose = PoseStamped()
    goal.pose.header.frame_id = "marker"
    goal.pose.header.stamp = rospy.Time.now()
    goal.pose.pose.position.x = x
    goal.pose.pose.position.y = y
    goal.pose.pose.position.z = 0.1
    goal.pose.pose.orientation.w = 1.0

    send_goal(client, goal)


def send_goal(client,  goal):

    rospy.loginfo('sending goal')
    client.send_goal(goal)

    client.wait_for_result()

    rospy.loginfo('got result')
    print client.get_result()
    print "status: " + client.get_goal_status_text()
    print "state: " + str(client.get_state())
    print


def main():

    l_client = actionlib.SimpleActionClient('/art/pr2/left_arm/pp', PickPlaceAction)
    l_client.wait_for_server()

    r_client = actionlib.SimpleActionClient('/art/pr2/right_arm/pp', PickPlaceAction)
    r_client.wait_for_server()

    goal = art_msgs.msg.PickPlaceGoal()
    goal.operation = goal.RESET
    send_goal(l_client, goal)
    send_goal(r_client, goal)

    get_ready(l_client)
    get_ready(r_client)

    rospy.sleep(2)  # time to store existing objects

    for client in [l_client, r_client]:

        pick_object_id(client, "3")
        place_object(client,  0.75, 0.5)
        get_ready(client)

    pick_object_id(l_client, "3")
    get_ready(l_client)

    pick_object_id(r_client, "4")
    place_object(r_client,  0.5, 0.5)
    get_ready(r_client)

    place_object(l_client,  0.75, 0.5)
    get_ready(l_client)

    rospy.loginfo("Done!")

if __name__ == '__main__':
    try:
        rospy.init_node('pp_client_py')
        main()
    except rospy.ROSInterruptException:
        print "program interrupted before completion"
