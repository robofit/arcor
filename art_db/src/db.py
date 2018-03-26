#!/usr/bin/env python

from art_msgs.msg import Program, ObjectType
from art_msgs.srv import getProgram, getProgramResponse, getProgramHeaders, getProgramHeadersResponse, \
    storeProgram, storeProgramResponse, getObjectType, getObjectTypeResponse, storeObjectType, storeObjectTypeResponse,\
    ProgramIdTrigger, ProgramIdTriggerResponse
import sys
import rospy
from art_utils import ProgramHelper
import threading

from mongodb_store.message_store import MessageStoreProxy


class ArtDB:

    def __init__(self):

        self.db = MessageStoreProxy()
        self.lock = threading.RLock()

        self.srv_get_program = rospy.Service('/art/db/program/get', getProgram, self.srv_get_program_cb)
        self.srv_get_program_headers = rospy.Service('/art/db/program_headers/get', getProgramHeaders, self.srv_get_program_headers_cb)
        self.srv_store_program = rospy.Service('/art/db/program/store', storeProgram, self.srv_store_program_cb)
        self.srv_delete_program = rospy.Service('/art/db/program/delete', ProgramIdTrigger, self.srv_delete_program_cb)
        self.srv_ro_set_program = rospy.Service('/art/db/program/readonly/set', ProgramIdTrigger, self.srv_ro_set_program_cb)
        self.srv_ro_clear_program = rospy.Service('/art/db/program/readonly/clear', ProgramIdTrigger,
                                                  self.srv_ro_clear_program_cb)

        self.srv_get_object = rospy.Service('/art/db/object_type/get', getObjectType, self.srv_get_object_cb)
        self.srv_store_object = rospy.Service('/art/db/object_type/store', storeObjectType, self.srv_store_object_cb)

        rospy.loginfo('art_db ready')

    def _program_set_ro(self, program_id, ro):

        with self.lock:

            name = "program:" + str(program_id)
            resp = ProgramIdTriggerResponse()
            resp.success = False

            try:
                prog = self.db.query_named(name, Program._type)[0]
            except rospy.ServiceException as e:
                resp.error = str(e)
                return resp

            prog.header.readonly = ro

            try:
                ret = self.db.update_named(name, prog, upsert=True)
            except rospy.ServiceException as e:
                resp.error = str(e)
                return resp

            resp.success = ret.success
            return resp

    def srv_ro_set_program_cb(self, req):

        return self._program_set_ro(req.program_id, True)

    def srv_ro_clear_program_cb(self, req):

        return self._program_set_ro(req.program_id, False)

    def srv_get_program_headers_cb(self, req):

        with self.lock:

            resp = getProgramHeadersResponse()

            programs = []

            try:
                programs = self.db.query(Program._type)
            except rospy.ServiceException as e:
                print "Service call failed: " + str(e)

            for prog in programs:
                if len(req.ids) == 0 or prog[0].header.id in req.ids:
                    resp.headers.append(prog[0].header)

            return resp

    def srv_delete_program_cb(self, req):

        with self.lock:

            resp = ProgramIdTriggerResponse()
            resp.success = False
            name = "program:" + str(req.program_id)

            try:

                meta = self.db.query_named(name, Program._type)[1]

                if self.db.delete(str(meta["_id"])):
                    resp.success = True
            except rospy.ServiceException as e:
                pass

            return resp

    def srv_get_program_cb(self, req):

        with self.lock:

            resp = getProgramResponse()
            resp.success = False
            name = "program:" + str(req.id)

            prog = None

            try:
                prog = self.db.query_named(name, Program._type)[0]
            except rospy.ServiceException as e:
                print "Service call failed: " + str(e)

            if prog is not None:

                resp.program = prog
                resp.success = True

            return resp

    def srv_store_program_cb(self, req):

        with self.lock:

            resp = storeProgramResponse()
            resp.success = False
            name = "program:" + str(req.program.header.id)

            try:
                prog = self.db.query_named(name, Program._type)[0]
            except rospy.ServiceException as e:
                print "Service call failed: " + str(e)
                return resp

            if prog is not None and prog.header.readonly:
                resp.error = "Readonly program."
                return resp

            ph = ProgramHelper()
            if not ph.load(req.program):

                resp.error = "Invalid program"
                return resp

            try:
                ret = self.db.update_named(name, req.program, upsert=True)
            except rospy.ServiceException as e:
                print "Service call failed: " + str(e)
                return resp

            resp.success = ret.success
            return resp

    def srv_get_object_cb(self, req):

        with self.lock:

            resp = getObjectTypeResponse()
            resp.success = False
            name = "object_type:" + str(req.name)

            try:
                object_type = self.db.query_named(name, ObjectType._type)[0]
            except rospy.ServiceException as e:
                print "Service call failed: " + str(e)
                return resp

            if object_type:

                resp.success = True
                resp.object_type = object_type
                return resp

            rospy.logerr("Unknown object type: " + req.name)
            return resp

    def srv_store_object_cb(self, req):

        with self.lock:

            resp = storeObjectTypeResponse()
            name = "object_type:" + str(req.object_type.name)

            try:
                ret = self.db.update_named(name, req.object_type, upsert=True)
            except rospy.ServiceException as e:
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
