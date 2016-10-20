#!/usr/bin/env python

import rospy
from art_msgs.srv import getProgram,  storeProgram,  startProgram

class ArtApiHelper():

    def __init__(self):

        # DB
        self.get_prog_srv = rospy.ServiceProxy('/art/db/program/get', getProgram)
        self.store_prog_srv = rospy.ServiceProxy('/art/db/program/store', storeProgram)

        # Brain
        self.start_program_srv = rospy.ServiceProxy('/art/brain/program/start', startProgram)

    def load_program(self,  prog_id):

        rospy.loginfo('Loading program: ' + str(prog_id))

        try:
            resp = self.get_prog_srv(prog_id)
            return resp.program
        except rospy.ServiceException, e:
            print "Service call failed: %s"%e

        return None

    def store_program(self,  prog):

            try:
                resp = self.store_prog_srv(prog)
            except rospy.ServiceException, e:
                print "Service call failed: %s"%e
                return False

            return True

    def start_program(self,  prog_id):

        try:
            resp = self.start_program_srv(prog_id)
        except rospy.ServiceException, e:
            print "Service call failed: %s"%e
            return False

        return True
