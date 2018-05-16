#!/usr/bin/env python

import sys
import rospy
from art_collision_env.int_collision_env import IntCollisionEnv
import os


def main(args):

    rospy.init_node('collision_env_node', anonymous=True)

    try:
        setup = os.environ["ARTABLE_SETUP"]
    except KeyError:
        rospy.logfatal("ARTABLE_SETUP has to be set.")
        return

    ce = IntCollisionEnv(setup, "marker")
    ce.load_from_db()
    ce.start()

    rospy.spin()


if __name__ == '__main__':
    try:
        main(sys.argv)
    except KeyboardInterrupt:
        print("Shutting down")
