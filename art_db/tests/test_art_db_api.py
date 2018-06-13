#!/usr/bin/env python

import sys
import unittest
import rospy
import rostest
from copy import deepcopy

from art_msgs.msg import Program, ProgramBlock, ProgramItem, ObjectType
from art_msgs.srv import getProgram, getProgramHeaders, storeProgram, getObjectType, storeObjectType
from shape_msgs.msg import SolidPrimitive
from geometry_msgs.msg import PoseStamped, PolygonStamped, Point32


class TestArtDb(unittest.TestCase):

    def setUp(self):

        rospy.wait_for_service('/art/db/program/store')
        rospy.wait_for_service('/art/db/program/get')
        rospy.wait_for_service('/art/db/object_type/store')
        rospy.wait_for_service('/art/db/object_type/get')

        self.get_object_srv = rospy.ServiceProxy('/art/db/object_type/get', getObjectType)
        self.store_object_srv = rospy.ServiceProxy('/art/db/object_type/store', storeObjectType)
        self.store_program_srv = rospy.ServiceProxy('/art/db/program/store', storeProgram)
        self.get_program_srv = rospy.ServiceProxy('/art/db/program/get', getProgram)
        self.get_program_headers_srv = rospy.ServiceProxy('/art/db/program_headers/get', getProgramHeaders)

    def test_object_type(self):

        ot = ObjectType()
        ot.name = "profile_test_1"
        ot.bbox.type = SolidPrimitive.BOX
        ot.bbox.dimensions.append(0.1)
        ot.bbox.dimensions.append(0.1)
        ot.bbox.dimensions.append(0.1)

        try:
            resp_store = self.store_object_srv(ot)
        except rospy.ServiceException:
            pass

        self.assertEquals(resp_store.success, True, "object_type_store")

        try:
            resp_get = self.get_object_srv(name="profile_test_1")
        except rospy.ServiceException:
            pass

        self.assertEquals(resp_get.success, True, "object_type_get")
        self.assertEquals(resp_get.object_type.name, "profile_test_1", "object_type_get")

    def test_invalid_object_type(self):

        try:
            resp_get = self.get_object_srv(name="profile_test_xy")
        except rospy.ServiceException:
            pass

        self.assertEquals(resp_get.success, False, "invalid_object_type_get")
        self.assertEquals(resp_get.object_type.name, "", "invalid_object_type_get")

    def test_program(self):

        prog = Program()
        prog.header.id = 999
        prog.header.name = "Test pick&place"

        pb = ProgramBlock()
        pb.id = 1  # can't be zero
        pb.name = "First block"
        pb.on_success = 1
        pb.on_failure = 0
        prog.blocks.append(pb)

        p = ProgramItem()
        p.id = 1
        p.on_success = 2
        p.on_failure = 0
        p.type = "GetReady"
        pb.items.append(deepcopy(p))

        p = ProgramItem()
        p.id = 2
        p.on_success = 3
        p.on_failure = 0
        p.type = "WaitForUser"
        pb.items.append(deepcopy(p))

        p = ProgramItem()
        p.id = 3
        p.on_success = 4
        p.on_failure = 0
        p.type = "PickFromFeeder"
        p.object.append("profile_20_60")
        pf = PoseStamped()
        pf.header.frame_id = "marker"
        pf.pose.position.x = 0.75
        pf.pose.position.y = 0.5
        p.pose.append(pf)
        pb.items.append(deepcopy(p))

        p = ProgramItem()
        p.id = 4
        p.on_success = 5
        p.on_failure = 0
        p.type = "PlaceToPose"
        p.ref_id.append(3)
        p.ref_id.append(5)
        pp = PoseStamped()
        pp.header.frame_id = "marker"
        pp.pose.position.x = 0.75
        pp.pose.position.y = 0.5
        p.pose.append(pp)
        pb.items.append(deepcopy(p))

        p = ProgramItem()
        p.id = 5
        p.on_success = 6
        p.on_failure = 0
        p.type = "PickFromFeeder"
        p.object.append("profile_20_60")
        pf = PoseStamped()
        pf.header.frame_id = "marker"
        pf.pose.position.x = 0.75
        pf.pose.position.y = 0.5
        p.pose.append(pf)
        pb.items.append(deepcopy(p))

        p = ProgramItem()
        p.id = 6
        p.on_success = 7
        p.on_failure = 0
        p.type = "GetReady"
        pb.items.append(deepcopy(p))

        p = ProgramItem()
        p.id = 7
        p.on_success = 8
        p.on_failure = 0
        p.type = "WaitUntilUserFinishes"
        pb.items.append(deepcopy(p))

        p = ProgramItem()
        p.id = 8
        p.on_success = 9
        p.on_failure = 0
        p.type = "PickFromPolygon"
        p.object.append("profile_20_60")
        pp = PolygonStamped()
        pp.header.frame_id = "marker"
        pp.polygon.points.append(Point32(0.4, 0.1, 0))
        pp.polygon.points.append(Point32(1.0, 0.1, 0))
        pp.polygon.points.append(Point32(1.0, 0.6, 0))
        pp.polygon.points.append(Point32(0.4, 0.6, 0))
        p.polygon.append(pp)
        pb.items.append(deepcopy(p))

        p = ProgramItem()
        p.id = 9
        p.on_success = 4
        p.on_failure = 0
        p.type = "PlaceToPose"
        p.ref_id.append(8)
        pp = PoseStamped()
        pp.header.frame_id = "marker"
        pp.pose.position.x = 0.75
        pp.pose.position.y = 0.5
        p.pose.append(pp)
        pb.items.append(deepcopy(p))

        try:
            resp_store = self.store_program_srv(program=prog)
        except rospy.ServiceException:
            pass

        self.assertEquals(resp_store.success, True, "program_store")

        try:
            resp_get = self.get_program_srv(id=999)
        except rospy.ServiceException:
            pass

        self.assertEquals(resp_get.success, True, "program_get")
        self.assertEquals(resp_get.program.header.id, 999, "program_get")

        try:
            resp_headers = self.get_program_headers_srv(ids=[999])
        except rospy.ServiceException:
            pass

        self.assertEquals(len(resp_headers.headers), 1, "program_headers_len")
        self.assertEquals(resp_headers.headers[0].id, 999, "program_headers_id")

    def test_invalid_program_get(self):

        try:
            resp_get = self.get_program_srv(id=1234)
        except rospy.ServiceException:
            pass

        self.assertEquals(resp_get.success, False, "invalid_program_get")

        try:
            resp_headers = self.get_program_headers_srv(ids=[1234])
        except rospy.ServiceException:
            pass

        self.assertEquals(len(resp_headers.headers), 0, "invalid_program_headers_len")

    def test_invalid_program_store(self):

        p = Program()

        try:
            resp_store = self.store_program_srv(program=p)
        except rospy.ServiceException:
            pass

        self.assertEquals(resp_store.success, False, "invalid_program_store")


if __name__ == '__main__':

    rostest.run('art_db', 'test_art_db', TestArtDb, sys.argv)
