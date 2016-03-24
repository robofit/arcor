#! /usr/bin/env python
import roslib; roslib.load_manifest('art_pr2_grasping')
import rospy
import actionlib
import art_pr2_grasping.msg
from geometry_msgs.msg import PoseStamped
from shape_msgs.msg import SolidPrimitive

def main():

    client = actionlib.SimpleActionClient('/pr2_pick_place/pp', art_pr2_grasping.msg.pickplaceAction)
    client.wait_for_server()
    
    rospy.loginfo('sending goal')

    # Creates a goal to send to the action server.
    goal = art_pr2_grasping.msg.pickplaceGoal()
    
    goal.id = "1"
    goal.operation = goal.PICK_AND_PLACE
    goal.arm = goal.LEFT_ARM

    goal.pose = PoseStamped()
    goal.pose.header.frame_id = "base_footprint"
    goal.pose.header.stamp = rospy.Time.now()
    goal.pose.pose.position.x = 0.7
    goal.pose.pose.position.y = 0.3
    goal.pose.pose.position.z = 0.74+0.13 # vyska stolu + pulka kosticky
    goal.pose.pose.orientation.x = 0.0
    goal.pose.pose.orientation.y = 0.0
    goal.pose.pose.orientation.z = 0.0
    goal.pose.pose.orientation.w = 1.0
    
    goal.pose2 = PoseStamped()
    goal.pose2.header.frame_id = "base_footprint"
    goal.pose2.header.stamp = rospy.Time.now()
    goal.pose2.pose.position.x = 0.6
    goal.pose2.pose.position.y = 0.1
    goal.pose2.pose.position.z = 0.74+0.13
    goal.pose2.pose.orientation.x = 0.0
    goal.pose2.pose.orientation.y = 0.0
    goal.pose2.pose.orientation.z = 0.707
    goal.pose2.pose.orientation.w = 0.707
    
    goal.bb = SolidPrimitive()
    goal.bb.type = SolidPrimitive.BOX
    goal.bb.dimensions.append(0.05)
    goal.bb.dimensions.append(0.05)
    goal.bb.dimensions.append(0.15)

    rospy.loginfo('sending goal')
    client.send_goal(goal)

    client.wait_for_result()

    rospy.loginfo('got result')
    print client.get_result()

if __name__ == '__main__':
    try:
        # Initializes a rospy node so that the SimpleActionClient can
        # publish and subscribe over ROS.
        rospy.init_node('pp_client_py')
        main()
    except rospy.ROSInterruptException:
        print "program interrupted before completion"
