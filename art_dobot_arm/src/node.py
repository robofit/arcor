#!/usr/bin/env python

import rospy
import time
from glob import glob

from pydobot import Dobot
from geometry_msgs.msg import Point
from std_msgs.msg import Bool

class DobotCommander(object):

    def dobot_cb(self, data):
        print("Tries to move robot to position: " + str(data.x) + " " + str(data.y) + " " + str(data.z))
        self.device.go(data.x, data.y, data.z)
        print("Movement done ")
        
    def suck_cb(self, data):
        if data.data:
            print("It sucks")
        else:
            print("It sucks not")
        self.device.suck(data.data)
        

    def __init__(self):
        

        available_ports = glob('/dev/tty*USB*')  # mask for OSX Dobot port
        if len(available_ports) == 0:
            print('no port found for Dobot Magician')
            exit(1)

        self.device = Dobot(port=available_ports[0])
        if self.device is None:
            exit(1)
    
        
        print("device.speed(100)")
        self.device.speed(100)
        
        
        self.sub = rospy.Subscriber("/dobot", Point, self.dobot_cb)
        self.sub2 = rospy.Subscriber("/suck", Bool, self.suck_cb)
        print("Dobot ready")
        
        """
        print("device.go(250.0, 0.0, 25.0)")
        device.go(250.0, 0.0, 25.0)
        print("device.speed(10)")
        device.speed(10)
        print("device.go(250.0, 0.0, 0.0)")
        device.go(250.0, 0.0, 0.0)
        device.go(200.0, 0.0, 0.0)
        device.go(250.0, 0.0, 0.0)
        print("time.sleep(2)")
        time.sleep(2)
        """
        
    def __del__(self):
        print("device.close()")
        self.device.close()
        
    


if __name__ == '__main__':
    rospy.init_node('art_dobot_arm', log_level=rospy.INFO)

    try:
        node = DobotCommander()

        rospy.spin()
    except rospy.ROSInterruptException:
        node = None
        pass
