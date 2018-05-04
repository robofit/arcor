#!/usr/bin/env python

import sys
import rospy
from art_collision_env.collision_env import CollisionEnv
import os


def main(args):

    rospy.init_node('collision_env_node', anonymous=True)

    try:
        CollisionEnv(setup=os.environ["ARTABLE_SETUP"])
    except KeyError:
        rospy.logfatal("ARTABLE_SETUP has to be set.")
        return

    rospy.spin()


if __name__ == '__main__':
    try:
        main(sys.argv)
    except KeyboardInterrupt:
        print("Shutting down")
