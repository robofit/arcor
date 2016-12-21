#!/usr/bin/env python

import sys
import unittest
import rospy
import rostest

from art_msgs.msg import Program,  ProgramBlock, ProgramItem,  ObjectType
from art_msgs.srv import getProgram,  storeProgram,   getObjectType,  storeObjectType
from shape_msgs.msg import SolidPrimitive


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

        p = Program()
        p.id = 999
        p.name = "Test pick&place"

        pb = ProgramBlock()
        pb.id = 1  # can't be zero
        pb.name = "First block"
        pb.on_success = 1
        pb.on_failure = 0
        p.blocks.append(pb)

        p0 = ProgramItem()
        p0.id = 1
        p0.on_success = 2
        p0.on_failure = 0
        p0.type = ProgramItem.GET_READY
        pb.items.append(p0)

        p1 = ProgramItem()
        p1.id = 2
        p1.on_success = 3
        p1.on_failure = 0
        p1.type = ProgramItem.WAIT
        p1.spec = ProgramItem.WAIT_FOR_USER
        pb.items.append(p1)

        p2 = ProgramItem()
        p2.id = 3
        p2.on_success = 4
        p2.on_failure = 0
        p2.type = ProgramItem.MANIP_PICK_PLACE
        p2.spec = ProgramItem.MANIP_TYPE
        p2.object = "profile"
        # TODO p2.pick_polygon
        p2.place_pose.header.frame_id = "marker"
        p2.place_pose.pose.position.x = 0.75
        p2.place_pose.pose.position.y = 0.5
        pb.items.append(p2)

        p3 = ProgramItem()
        p3.id = 4
        p3.on_success = 5
        p3.on_failure = 0
        p3.type = ProgramItem.WAIT
        p3.spec = ProgramItem.WAIT_UNTIL_USER_FINISHES
        pb.items.append(p3)

        p4 = ProgramItem()
        p4.id = 5
        p4.on_success = 0
        p4.on_failure = 0
        p4.type = ProgramItem.MANIP_PICK_PLACE
        p4.spec = ProgramItem.MANIP_TYPE
        p4.object = "profile"
        # TODO p4.pick_polygon?
        p4.pick_pose = p2.place_pose
        p4.place_pose.header.frame_id = "marker"
        p4.place_pose.pose.position.x = 0.25
        p4.place_pose.pose.position.y = 0.5
        pb.items.append(p4)

        try:
            resp_store = self.store_program_srv(program=p)
        except rospy.ServiceException:
            pass

        self.assertEquals(resp_store.success, True, "program_store")

        try:
            resp_get = self.get_program_srv(id=999)
        except rospy.ServiceException:
            pass

        self.assertEquals(resp_get.success, True, "program_get")
        self.assertEquals(resp_get.program.id, 999, "program_get")

    def test_invalid_program_get(self):

        try:
            resp_get = self.get_program_srv(id=1234)
        except rospy.ServiceException:
            pass

        self.assertEquals(resp_get.success, False, "invalid_program_get")

    def test_invalid_program_store(self):

        p = Program()

        try:
            resp_store = self.store_program_srv(program=p)
        except rospy.ServiceException:
            pass

        self.assertEquals(resp_store.success, False, "invalid_program_store")


if __name__ == '__main__':

    rostest.run('art_db', 'test_art_db', TestArtDb, sys.argv)
