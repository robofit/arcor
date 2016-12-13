#!/usr/bin/env python

import rospy
import time
import numpy as np

import pycopia.OS.Linux.Input as input

from geometry_msgs.msg import PoseStamped

class ArtTouchDriver:  

    

    def __init__(self):
        self.x = 0
        self.y = 0
        self.touch = False
        self.touch_id = -1
        self.device = input.EventDevice("/dev/input/event17")
        self.point_pub = rospy.Publisher("/art/interface/touchtable/single", PoseStamped, queue_size=1)
        pass
        
    def process(self):
        #print 1 if self.device._eventq else 0
        event = self.device.read()
        while True:
           
            if not self.touch and event.evtype == 3 and event.code == 57 and event.value >= 0:
                self.touch = True
                self.touch_id = event.value
                print "new id: " + str(event.value)
                
            elif self.touch and event.evtype == 3 and event.code == 57 and event.value < 0:
                self.touch = False
                print "lost id: " + str(self.touch_id)
                self.touch_id = event.value 
                
            elif self.touch and event.evtype == 3 and event.code == 0: 
                # x position
                self.x = event.value
                
            elif self.touch and event.evtype == 3 and event.code == 1:
                # y position
                self.y = event.value
            elif event.evtype == 0:
                pose = PoseStamped()
                pose.pose.position.x = self.x
                pose.pose.position.y = self.y
                self.point_pub.publish(pose)
            else:
                pass
                #print event
            
            if not self.device._eventq:
                break
            event = self.device.read()
            
        
        
        
        

if __name__ == '__main__':
    rospy.init_node('art_touch_driver')

    rospy.loginfo('Waiting for other nodes to come up...')
   
    rospy.loginfo('Ready!')

    try:
        node = ArtTouchDriver()
        rate = rospy.Rate(25)
        
        while not rospy.is_shutdown():
            node.process()
            rate.sleep()
    except rospy.ROSInterruptException:
        pass
