#!/usr/bin/env python

import rospy
from std_msgs.msg import Bool, Header
from art_msgs.srv import getProgram, getProgramResponse, getProgramHeaders, getProgramHeadersResponse, \
    storeProgram, storeProgramResponse, getObjectType, getObjectTypeResponse, storeObjectType, storeObjectTypeResponse,\
    ProgramIdTrigger, ProgramIdTriggerResponse, GetCollisionPrimitives, GetCollisionPrimitivesResponse,\
    AddCollisionPrimitive, AddCollisionPrimitiveResponse, ClearCollisionPrimitives, ClearCollisionPrimitivesResponse

from art_msgs.msg import Program, ProgramBlock, ProgramItem, ProgramHeader, ObjectType
from geometry_msgs.msg import PoseStamped, PolygonStamped, Pose, Point, Quaternion, Polygon, Point32
from shape_msgs.msg import SolidPrimitive


class FakeCalibration:

    calib_table = False

    def __init__(self):
        self.srv_get_program = rospy.Service('/art/db/program/get', getProgram, self.srv_get_program_cb)
        self.srv_get_program_headers = rospy.Service('/art/db/program_headers/get',
                                                     getProgramHeaders,
                                                     self.srv_get_program_headers_cb)

        self.srv_store_program = rospy.Service('/art/db/program/store', storeProgram, self.srv_store_program_cb)
        self.srv_delete_program = rospy.Service('/art/db/program/delete', ProgramIdTrigger, self.srv_delete_program_cb)
        self.srv_ro_set_program = rospy.Service('/art/db/program/readonly/set',
                                                ProgramIdTrigger,
                                                self.srv_ro_set_program_cb)
        self.srv_ro_clear_program = rospy.Service('/art/db/program/readonly/clear', ProgramIdTrigger,
                                                  self.srv_ro_clear_program_cb)

        self.srv_get_object = rospy.Service('/art/db/object_type/get', getObjectType, self.srv_get_object_cb)
        self.srv_store_object = rospy.Service('/art/db/object_type/store', storeObjectType, self.srv_store_object_cb)

        self.srv_get_collision_primitives = rospy.Service('/art/db/collision_primitives/get', GetCollisionPrimitives,
                                                          self.srv_get_collision_primitives_cb)
        self.srv_add_collision_primitive = rospy.Service('/art/db/collision_primitives/add', AddCollisionPrimitive,
                                                         self.srv_add_collision_primitive_cb)
        self.srv_clear_collision_primitive = rospy.Service('/art/db/collision_primitives/clear',
                                                           ClearCollisionPrimitives,
                                                           self.srv_clear_collision_primitives_cb)

    def srv_get_program_cb(self, request):
        resp = getProgramResponse()
        prog = Program()
        prog.header.id = request.id
        prog.header.name = "Demo"

        pb = ProgramBlock()
        pb.id = 1
        pb.name = "Demo"
        pb.on_success = 1
        pb.on_failure = 0
        prog.blocks.append(pb)

        pi = ProgramItem()
        pi.type = "PickFromBin"
        pi.object.append("bin")
        pi.id = 1
        pi.on_success = 2
        pi.on_failure = 1  # nebo GetReady a konec?
        pi.do_not_clear.append("object")
        pb.items.append(pi)

        pi = ProgramItem()
        pi.type = "VisualInspection"
        pi.ref_id.append(1)
        pi.pose.append(PoseStamped(header=Header(frame_id="marker"),
                                   pose=Pose(position=Point(x=1), orientation=Quaternion(w=1))))
        pi.id = 2
        pi.on_success = 3
        pi.on_failure = 4
        pb.items.append(pi)

        pi = ProgramItem()
        pi.type = "PlaceToContainer"
        pi.name = "Place OK"
        pi.id = 3
        pi.on_success = 1
        pi.object.append("blue_container")
        pi.polygon.append(PolygonStamped(header=Header(frame_id="marker"),
                                         polygon=Polygon(points=[Point32(x=0, y=0),
                                                                 Point32(x=1, y=0),
                                                                 Point32(x=1, y=1),
                                                                 Point32(x=0, y=1)])))
        pi.ref_id.append(1)
        pb.items.append(pi)

        pi = ProgramItem()
        pi.type = "PlaceToContainer"
        pi.name = "Place NOK"
        pi.id = 4
        pi.on_success = 1
        pi.object.append("blue_container")
        pi.polygon.append(PolygonStamped(header=Header(frame_id="marker"),
                                         polygon=Polygon(points=[Point32(x=0, y=0),
                                                                 Point32(x=1, y=0),
                                                                 Point32(x=1, y=1),
                                                                 Point32(x=0, y=1)])))
        pi.ref_id.append(1)
        pb.items.append(pi)
        resp.program = prog
        resp.success = True
        return resp

    def srv_get_program_headers_cb(self, request):
        response = getProgramHeadersResponse()
        response.headers.append(ProgramHeader(id=1, name="asdf", description="asdf", readonly=False))
        return response

    def srv_store_program_cb(self, request):
        return storeProgramResponse(success=True)

    def srv_delete_program_cb(self, request):
        return ProgramIdTriggerResponse(success=True)

    def srv_ro_set_program_cb(self, request):
        return ProgramIdTriggerResponse(success=True)

    def srv_ro_clear_program_cb(self, request):
        return ProgramIdTriggerResponse(success=True)

    def srv_get_object_cb(self, request):
        return getObjectTypeResponse(success=True,
                                     object_type=ObjectType(
                                         name=request.name,
                                         container=True,
                                         bbox=SolidPrimitive(type=1, dimensions=[0.1, 0.1, 0.1])))

    def srv_store_object_cb(self, request):
        return storeObjectTypeResponse(success=True)

    def srv_get_collision_primitives_cb(self, request):
        return GetCollisionPrimitivesResponse()

    def srv_add_collision_primitive_cb(self, request):
        return AddCollisionPrimitiveResponse(success=True)

    def srv_clear_collision_primitives_cb(self, request):
        return ClearCollisionPrimitivesResponse(success=True)


if __name__ == '__main__':
    rospy.init_node('fake_db')
    ''',log_level=rospy.DEBUG'''

    try:
        node = FakeCalibration()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
