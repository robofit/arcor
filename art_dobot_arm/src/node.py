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

        self.device = Dobot(port=available_ports[0], verbose=False)  # type: Dobot

        if self.device is None:
             exit(1)

        print("device.speed(100)")
        self.device.speed(100)

        self.sub = rospy.Subscriber("/dobot", Point, self.dobot_cb)
        self.sub2 = rospy.Subscriber("/suck", Bool, self.suck_cb)
        print("Dobot ready")
        return
        print("device.go(250.0, 0.0, 25.0)")
        self.device.go(215.0, 0.0, -50.0)
        print("device.speed(10)")
        self.device.speed(10)
        print("device.go(250.0, 0.0, 0.0)")
        self.device.go(225.0, 0.0, -50.0)
        time.sleep(1)
        self.device.go(235.0, 0.0, -50.0)
        time.sleep(1)
        self.device.go(245.0, 0.0, -50.0)
        time.sleep(1)


        print("time.sleep(2)")
        time.sleep(2)

    def __del__(self):
        print("device.close()")
        self.device.close()

    def get_dobot_pose(self):
        print("pydobot: x:%03.1f y:%03.1f z:%03.1f r:%03.1f j1:%03.1f j2:%03.1f j3:%03.1f j4:%03.1f" %
              (self.device.x, self.device.y, self.device.z, self.device.r, self.device.j1,
               self.device.j2, self.device.j3, self.device.j4))


if __name__ == '__main__':
    rospy.init_node('art_dobot_arm', log_level=rospy.INFO)

    try:
        node = DobotCommander()
        r = rospy.Rate(5)
        while not rospy.is_shutdown():
            node.get_dobot_pose()
            r.sleep()
            pass
        node.device.close()
        node = None
    except rospy.ROSInterruptException:

        node.device.close()
        node = None
        pass
