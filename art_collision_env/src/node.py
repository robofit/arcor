#!/usr/bin/env python

import sys
import rospy
from art_collision_env import CollisionEnv


def main(args):

    rospy.init_node('collision_env_node', anonymous=True)
    CollisionEnv()
    rospy.spin()


if __name__ == '__main__':
    try:
        main(sys.argv)
    except KeyboardInterrupt:
        print("Shutting down")
