#!/usr/bin/env python

from art_msgs.msg import Program,  ObjectType
from art_msgs.srv import getProgram,  getProgramResponse,  storeProgram,  storeProgramResponse,  getObjectType,  getObjectTypeResponse,  storeObjectType,  storeObjectTypeResponse
import sys
import rospy
from art_utils import ProgramHelper

from mongodb_store.message_store import MessageStoreProxy


class ArtDB:

    def __init__(self):

        self.db = MessageStoreProxy()

        self.srv_get_program = rospy.Service('/art/db/program/get', getProgram, self.srv_get_program_cb)
        self.srv_store_program = rospy.Service('/art/db/program/store', storeProgram, self.srv_store_program_cb)

        self.srv_get_object = rospy.Service('/art/db/object_type/get', getObjectType, self.srv_get_object_cb)
        self.srv_store_object = rospy.Service('/art/db/object_type/store', storeObjectType, self.srv_store_object_cb)

        rospy.loginfo('art_db ready')

    def srv_get_program_cb(self,  req):

        resp = getProgramResponse()
        name = "program:" + str(req.id)

        try:
            resp.program = self.db.query_named(name, Program._type)[0]
            return resp
        except rospy.ServiceException, e:
            print "Service call failed: " + str(e)
            return None

    def srv_store_program_cb(self,  req):

        resp = storeProgramResponse()

        ph = ProgramHelper()
        if not ph.load(req.program):

            resp.success = False
            resp.error = "Invalid program"
            return resp

        name = "program:" + str(req.program.id)

        try:
            ret = self.db.update_named(name,  req.program,  upsert=True)
        except rospy.ServiceException, e:
            print "Service call failed: " + str(e)
            resp.success = False
            return resp

        resp.success = ret.success
        return resp

    def srv_get_object_cb(self,  req):

        resp = getObjectTypeResponse()
        name = "object_type:" + str(req.name)

        try:
            resp.object_type = self.db.query_named(name, ObjectType._type)[0]
            return resp
        except rospy.ServiceException, e:
            print "Service call failed: " + str(e)
            return None

    def srv_store_object_cb(self,  req):

        resp = storeObjectTypeResponse()
        name = "object_type:" + str(req.object_type.name)

        try:
            ret = self.db.update_named(name,  req.object_type,  upsert=True)
        except rospy.ServiceException, e:
            print "Service call failed: " + str(e)
            resp.success = False
            return resp

        resp.success = ret.success
        return resp


def main(args):

    rospy.init_node('art_db')
    ArtDB()
    rospy.spin()

if __name__ == '__main__':
    try:
        main(sys.argv)
    except KeyboardInterrupt:
        print("Shutting down")
        
        
