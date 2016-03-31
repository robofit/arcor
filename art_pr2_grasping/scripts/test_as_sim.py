#! /usr/bin/env python
import roslib; roslib.load_manifest('art_pr2_grasping')
import rospy
import actionlib
import art_msgs.msg
from geometry_msgs.msg import PoseStamped
from shape_msgs.msg import SolidPrimitive
import random

def main():

    client = actionlib.SimpleActionClient('/pr2_pick_place/pp', art_msgs.pickplaceAction)
    client.wait_for_server()

    rospy.loginfo('sending goal')

    goal = art_msgs.msg.pickplaceGoal()
    
    goal.id = "my_object"
    goal.operation = goal.PICK_AND_PLACE
    goal.z_axis_angle_increment = (2*3.14)/360*180
    #goal.z_axis_angle_increment = 0
    goal.keep_orientation = True
    
    arms = [goal.LEFT_ARM, goal.RIGHT_ARM]
    
    goal.arm = random.choice(arms)
    
    goal.bb = SolidPrimitive()
    goal.bb.type = SolidPrimitive.BOX
    goal.bb.dimensions.append(0.2)
    goal.bb.dimensions.append(0.05)
    goal.bb.dimensions.append(0.05)

    goal.pose = PoseStamped()
    goal.pose.header.frame_id = "base_footprint"
    goal.pose.header.stamp = rospy.Time.now()
    goal.pose.pose.position.x = random.uniform(0.4, 0.7)
    goal.pose.pose.position.y = random.uniform(-0.5, 0.5)
    goal.pose.pose.position.z = 0.74 + goal.bb.dimensions[2]/2 # vyska stolu + pulka kosticky
    goal.pose.pose.orientation.x = 0.0
    goal.pose.pose.orientation.y = 0.0
    goal.pose.pose.orientation.z = 0.0
    goal.pose.pose.orientation.w = 1.0
    
    goal.pose2 = PoseStamped()
    goal.pose2.header.frame_id = "base_footprint"
    goal.pose2.header.stamp = rospy.Time.now()
    goal.pose2.pose.position.x = random.uniform(0.4, 0.7)
    goal.pose2.pose.position.y = random.uniform(-0.5, 0.5)
    goal.pose2.pose.position.z = 0.74 + goal.bb.dimensions[2]/2
    goal.pose2.pose.orientation.x = 0.0
    goal.pose2.pose.orientation.y = 0.0
    goal.pose2.pose.orientation.z = 0.0
    goal.pose2.pose.orientation.w = 1.0

    rospy.loginfo('sending goal')
    client.send_goal(goal)

    client.wait_for_result()

    rospy.loginfo('got result')
    print client.get_result()
    print "status: " + client.get_goal_status_text()
    print "state: " + str(client.get_state())

if __name__ == '__main__':
    try:
        rospy.init_node('pp_client_py')
        main()
    except rospy.ROSInterruptException:
        print "program interrupted before completion"
