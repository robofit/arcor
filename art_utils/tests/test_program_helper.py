#!/usr/bin/env python

import rospy
import unittest
import rostest
from art_utils import ProgramHelper
from art_msgs.msg import Program,  ProgramBlock,  ProgramItem
import sys
from copy import deepcopy


class TestProgramHelper(unittest.TestCase):

    def setUp(self):

        # TODO add more blocks

        self.prog = Program()
        self.ph = ProgramHelper()

        self.prog.header.id = 666
        self.prog.header.name = "Basic pick&place"

        pb = ProgramBlock()
        pb.id = 1  # can't be zero
        pb.name = "First block"
        pb.on_success = 1
        pb.on_failure = 0
        self.prog.blocks.append(pb)

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
        p4.pick_pose = p2.place_pose
        p4.place_pose.header.frame_id = "marker"
        p4.place_pose.pose.position.x = 0.25
        p4.place_pose.pose.position.y = 0.5
        pb.items.append(p4)

    def test_empty_program(self):

        res = self.ph.load(Program())
        self.assertEquals(res, False, "empty program")

    def test_empty_block(self):

        prog = Program()
        prog.blocks.append(ProgramBlock())
        res = self.ph.load(prog)
        self.assertEquals(res, False, "empty block")

    def test_invalid_block_id(self):

        prog = deepcopy(self.prog)
        prog.blocks[0].id = 0
        res = self.ph.load(prog)
        self.assertEquals(res, False, "invalid block id")

    def test_invalid_item_id(self):

        prog = deepcopy(self.prog)
        prog.blocks[0].items[0].id = 0
        res = self.ph.load(prog)
        self.assertEquals(res, False, "invalid item id")

    def test_invalid_item_on_success(self):

        prog = deepcopy(self.prog)
        prog.blocks[0].items[1].on_success = 1234
        res = self.ph.load(prog)
        self.assertEquals(res, False, "invalid item on_success")

    def test_invalid_item_on_failure(self):

        prog = deepcopy(self.prog)
        prog.blocks[0].items[1].on_failure = 1234
        res = self.ph.load(prog)
        self.assertEquals(res, False, "invalid item on_failure")

    def test_valid_program(self):

        res = self.ph.load(self.prog)
        self.assertEquals(res, True, "valid program")

    def test_on_success(self):

        res = self.ph.load(self.prog)
        self.assertEquals(res, True, "on_success")

        (block_id,  item_id) = self.ph.get_id_on_success(1,  2)
        self.assertEquals(block_id, 1, "on_success - block_id")
        self.assertEquals(item_id, 3, "on_success - item_id")

    def test_on_failure(self):

        res = self.ph.load(self.prog)
        self.assertEquals(res, True, "on_failure")

        (block_id,  item_id) = self.ph.get_id_on_failure(1,  4)
        self.assertEquals(block_id, 0, "on_failure - block_id")
        self.assertEquals(item_id, 0, "on_failure - item_id")

    def test_get_item_msg(self):

        res = self.ph.load(self.prog)
        self.assertEquals(res, True, "get_item_msg")

        msg = self.ph.get_item_msg(1,  4)
        self.assertEquals(isinstance(msg,  ProgramItem), True, "get_item_msg - type")
        self.assertEquals(msg.id, 4, "get_item_msg - id")

    def test_get_block_msg(self):

        res = self.ph.load(self.prog)
        self.assertEquals(res, True, "get_block_msg")

        msg = self.ph.get_block_msg(1)
        self.assertEquals(isinstance(msg,  ProgramBlock), True, "get_block_msg - type")
        self.assertEquals(msg.id, 1, "get_block_msg - id")

    def test_get_first_block_id(self):

        res = self.ph.load(self.prog)
        self.assertEquals(res, True, "get_first_block_id")

        block_id = self.ph.get_first_block_id()
        self.assertEquals(block_id, self.prog.blocks[0].id, "get_first_block_id")

    def test_get_first_item_id(self):

        res = self.ph.load(self.prog)
        self.assertEquals(res, True, "get_first_block_id")

        (block_id,  item_id) = self.ph.get_first_item_id()
        self.assertEquals(block_id, self.prog.blocks[0].id, "get_first_item_id - block")
        self.assertEquals(item_id, self.prog.blocks[0].items[0].id, "get_first_item_id - item")

    def test_get_program_id(self):

        res = self.ph.load(self.prog)
        self.assertEquals(res, True, "get_program_id")

        prog_id = self.ph.get_program_id()
        self.assertEquals(prog_id, 666, "get_program_id - id")

    def test_template(self):

        self.ph.load(self.prog)
        self.assertEquals(self.ph.program_learned(), True, "test_template")
        self.ph.load(self.prog, True)
        self.assertEquals(self.ph.program_learned(), False, "test_template")

if __name__ == '__main__':

    rospy.init_node('test_node')
    rostest.run('art_utils', 'test_program_helper', TestProgramHelper, sys.argv)
