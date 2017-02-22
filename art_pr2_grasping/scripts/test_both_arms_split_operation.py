#! /usr/bin/env python

import rospy
import actionlib
import art_msgs.msg
from art_msgs.msg import PickPlaceAction

def main():

    l_client = actionlib.SimpleActionClient('/art/pr2/left_arm/pp', PickPlaceAction)
    l_client.wait_for_server()

    r_client = actionlib.SimpleActionClient('/art/pr2/left_arm/pp', PickPlaceAction)
    r_client.wait_for_server()

    arr = InstancesArray()
    arr.header.frame_id = "base_footprint"
    arr.header.stamp = rospy.Time.now()

    obj = ObjInstance()
    obj.object_id = "my_object"
    obj.pose.position.x = random.uniform(0.4, 0.7)
    obj.pose.position.y = random.uniform(-0.2, 0.5)
    obj.pose.position.z = 0.74 + 0.1 # vyska stolu + pulka kosticky
    obj.pose.orientation.x = 0.0
    obj.pose.orientation.y = 0.0
    obj.pose.orientation.z = 0.0
    obj.pose.orientation.w = 1.0

    obj.bbox = SolidPrimitive()
    obj.bbox.type = SolidPrimitive.BOX
    obj.bbox.dimensions.append(0.05)
    obj.bbox.dimensions.append(0.05)
    obj.bbox.dimensions.append(0.2)

    arr.instances.append(obj)
    arr.instances.append(getRandomObject())
    arr.instances.append(getRandomObject())

    pub.publish(arr)
    rospy.sleep(2.0)

    goal = art_msgs.msg.pickplaceGoal()

    goal.id = "my_object"
    goal.operation = goal.PICK_AND_PLACE
    goal.z_axis_angle_increment = (2*3.14)/360*90
    goal.keep_orientation = False

    goal.place_pose = PoseStamped()
    goal.place_pose.header.frame_id = "base_footprint"
    goal.place_pose.header.stamp = rospy.Time.now()
    goal.place_pose.pose.position.x = random.uniform(0.4, 0.7)
    goal.place_pose.pose.position.y = random.uniform(-0.2, 0.5)
    goal.place_pose.pose.position.z = 0.74 + 0.1
    goal.place_pose.pose.orientation.x = 0.0
    goal.place_pose.pose.orientation.y = 0.0
    goal.place_pose.pose.orientation.z = 0.0
    goal.place_pose.pose.orientation.w = 1.0

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
