#! /usr/bin/env python

import rospy
import actionlib
from art_msgs.msg import PickPlaceAction, PickPlaceGoal, PickPlaceResult
from geometry_msgs.msg import PoseStamped


def pick_object_id(client,  object_id):

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
    goal.pose.pose.position.z = 0.1
    goal.pose.pose.orientation.w = 1.0

    return send_goal(client, goal)


def send_goal(client,  goal):

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

    l_client = actionlib.SimpleActionClient('/art/pr2/left_arm/pp', PickPlaceAction)
    l_client.wait_for_server()

    r_client = actionlib.SimpleActionClient('/art/pr2/right_arm/pp', PickPlaceAction)
    r_client.wait_for_server()

    # goal = PickPlaceGoal()
    # goal.operation = goal.RESET
    # send_goal(l_client, goal)
    # send_goal(r_client, goal)

    if get_ready(l_client) != PickPlaceResult.SUCCESS:
        return
        
    if get_ready(r_client) != PickPlaceResult.SUCCESS:
        return

    rospy.sleep(2)  # time to store existing objects

    #for client in [l_client, r_client]:

     #   if pick_object_id(client, "3") != PickPlaceResult.SUCCESS:
      #      return
        
       # if place_object(client,  0.75, 0.4) != PickPlaceResult.SUCCESS:
        #    return
        
       # if get_ready(client) != PickPlaceResult.SUCCESS:
        #    return
            
        #rospy.sleep(5)

    if pick_object_id(l_client, "3")  != PickPlaceResult.SUCCESS:
        return
        
    if get_ready(l_client)  != PickPlaceResult.SUCCESS:
        return

    if pick_object_id(r_client, "4")  != PickPlaceResult.SUCCESS:
        return
        
    if place_object(r_client,  0.5, 0.4)  != PickPlaceResult.SUCCESS:
        return
    
    if get_ready(r_client)  != PickPlaceResult.SUCCESS:
        return

    if place_object(l_client,  0.75, 0.4) != PickPlaceResult.SUCCESS:
        return
    
    if get_ready(l_client)  != PickPlaceResult.SUCCESS:
        return

    rospy.loginfo("Done!")

if __name__ == '__main__':
    try:
        rospy.init_node('pp_client_py')
        main()
    except rospy.ROSInterruptException:
        print "program interrupted before completion"
