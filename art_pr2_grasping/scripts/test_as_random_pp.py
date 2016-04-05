#! /usr/bin/env python
import roslib; roslib.load_manifest('art_pr2_grasping')
import rospy
import actionlib
import art_msgs.msg
from geometry_msgs.msg import PoseStamped
from shape_msgs.msg import SolidPrimitive
import random
from art_object_recognizer_msgs.msg import InstancesArray, ObjInstance

def main():

    pub = rospy.Publisher("/art_object_detector/object_filtered", InstancesArray)
    
    client = actionlib.SimpleActionClient('/pr2_pick_place/pp', art_msgs.msg.pickplaceAction)
    client.wait_for_server()
    
    arr = InstancesArray()
    arr.header.frame_id = "base_footprint"
    arr.header.stamp = rospy.Time.now()
    
    obj = ObjInstance()
    obj.object_id = "my_object"
    obj.pose.position.x = random.uniform(0.4, 0.7)
    obj.pose.position.y = random.uniform(-0.5, 0.5)
    obj.pose.position.z = 0.74 + 0.05/2 # vyska stolu + pulka kosticky
    obj.pose.orientation.x = 0.0
    obj.pose.orientation.y = 0.0
    obj.pose.orientation.z = 0.0
    obj.pose.orientation.w = 1.0
    
    obj.bbox = SolidPrimitive()
    obj.bbox.type = SolidPrimitive.BOX
    obj.bbox.dimensions.append(0.2)
    obj.bbox.dimensions.append(0.05)
    obj.bbox.dimensions.append(0.05)

    arr.instances.append(obj)
    
    obj2 = ObjInstance()
    obj2.object_id = "another_object"
    obj2.pose.position.x = random.uniform(0.4, 0.7)
    obj2.pose.position.y = random.uniform(-0.5, 0.5)
    obj2.pose.position.z = 0.74 + 0.05/2 # vyska stolu + pulka kosticky
    obj2.pose.orientation.x = 0.0
    obj2.pose.orientation.y = 0.0
    obj2.pose.orientation.z = 0.0
    obj2.pose.orientation.w = 1.0
    
    obj2.bbox = SolidPrimitive()
    obj2.bbox.type = SolidPrimitive.BOX
    obj2.bbox.dimensions.append(0.2)
    obj2.bbox.dimensions.append(0.05)
    obj2.bbox.dimensions.append(0.05)
    
    arr.instances.append(obj2)
    
    pub.publish(arr)
    #pub.publish(arr)
    #pub.publish(arr)

    goal = art_msgs.msg.pickplaceGoal()
    
    goal.id = "my_object"
    goal.operation = goal.PICK_AND_PLACE
    goal.z_axis_angle_increment = (2*3.14)/360*180
    #goal.z_axis_angle_increment = 0
    goal.keep_orientation = False
    
    #arms = [goal.LEFT_ARM, goal.RIGHT_ARM]
    #goal.arm = random.choice(arms)
    goal.arm = goal.LEFT_ARM
    
    goal.place_pose = PoseStamped()
    goal.place_pose.header.frame_id = "base_footprint"
    goal.place_pose.header.stamp = rospy.Time.now()
    goal.place_pose.pose.position.x = random.uniform(0.4, 0.7)
    goal.place_pose.pose.position.y = random.uniform(-0.5, 0.5)
    goal.place_pose.pose.position.z = 0.74 + 0.05/2
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
