#!/usr/bin/env python

import rospy
import time
from glob import glob

from pydobot import Dobot
from geometry_msgs.msg import Point
from std_msgs.msg import Bool

class DobotTest(object):

   
        

    def __init__(self):
        
        
        
        self.pub = rospy.Publisher("/dobot", Point)
        self.pub2 = rospy.Publisher("/suck", Bool)

        point = Point()
        suck = Bool()
        point.x = 250
        point.y = 0
        point.z = 50
        self.pub.publish(point)
        rospy.sleep(2)
        point.x = 250
        point.y = 0
        point.z = 9
        self.pub.publish(point)
        rospy.sleep(1)
        suck.data = True
        self.pub2.publish(suck)
        rospy.sleep(1)
        point.x = 250
        point.y = 0
        point.z = 50
        self.pub.publish(point)
        rospy.sleep(1)
        point.x = 250
        point.y = 100
        point.z = 50
        self.pub.publish(point)
        rospy.sleep(1)
        point.x = 250
        point.y = 100
        point.z = 9
        self.pub.publish(point)
        rospy.sleep(1)
        suck.data = False
        self.pub2.publish(suck)
        rospy.sleep(1)
        point.x = 250
        point.y = 100
        point.z = 50
        self.pub.publish(point)
        rospy.sleep(1)
        point.x = 250
        point.y = 0
        point.z = 50
        self.pub.publish(point)
        
        
    def __del__(self):
        print("device.close()")
        
        
    


if __name__ == '__main__':
    rospy.init_node('art_dobot_arm_test', log_level=rospy.INFO)

    try:
        node = DobotTest()

        
    except rospy.ROSInterruptException:
        node = None
        pass
