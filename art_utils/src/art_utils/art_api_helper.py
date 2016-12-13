#!/usr/bin/env python

import rospy
from art_msgs.srv import getProgram, storeProgram, startProgram, getObjectType


class ArtApiHelper():

    def __init__(self):

        # DB
        self.get_prog_srv = rospy.ServiceProxy('/art/db/program/get', getProgram)
        self.store_prog_srv = rospy.ServiceProxy('/art/db/program/store', storeProgram)
        self.get_obj_type_srv = rospy.ServiceProxy('/art/db/object_type/get', getObjectType)

        # Brain
        self.start_program_srv = rospy.ServiceProxy('/art/brain/program/start', startProgram)

    def wait_for_api(self):

        self.get_prog_srv.wait_for_service()
        self.store_prog_srv.wait_for_service()
        self.start_program_srv.wait_for_service()
        self.get_obj_type_srv.wait_for_service()

    def load_program(self, prog_id):

        rospy.loginfo('Loading program: ' + str(prog_id))

        try:
            resp = self.get_prog_srv(prog_id)
            return resp.program
        except rospy.ServiceException, e:
            print "Service call failed: %s" % e

        return None

    def store_program(self, prog):

        try:
            self.store_prog_srv(prog)
        except rospy.ServiceException, e:
            print "Service call failed: %s" % e
            return False

        return True

    def start_program(self, prog_id):

        try:
            self.start_program_srv(prog_id)
        except rospy.ServiceException, e:
            print "Service call failed: %s" % e
            return False

        return True

    def get_object_type(self, name):

        try:
            resp = self.get_obj_type_srv(name)
            return resp.object_type
        except rospy.ServiceException, e:
            print "Service call failed: %s" % e

        return None
