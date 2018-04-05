#!/usr/bin/env python

import rospy
import unittest
import rostest
from art_utils import ProgramHelper
from art_msgs.msg import Program, ProgramBlock, ProgramItem
import sys
from copy import deepcopy
from geometry_msgs.msg import Point32, PoseStamped, PolygonStamped


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

        p = ProgramItem()
        p.id = 1
        p.on_success = 2
        p.on_failure = 0
        p.type = ProgramItem.GET_READY
        pb.items.append(deepcopy(p))

        p = ProgramItem()
        p.id = 2
        p.on_success = 3
        p.on_failure = 0
        p.type = ProgramItem.WAIT_FOR_USER
        pb.items.append(deepcopy(p))

        p = ProgramItem()
        p.id = 3
        p.on_success = 4
        p.on_failure = 0
        p.type = ProgramItem.PICK_FROM_FEEDER
        p.object.append("profile")
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
        p.type = ProgramItem.PLACE_TO_POSE
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
        p.type = ProgramItem.PICK_FROM_FEEDER
        p.object.append("profile")
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
        p.type = ProgramItem.GET_READY
        pb.items.append(deepcopy(p))

        p = ProgramItem()
        p.id = 7
        p.on_success = 8
        p.on_failure = 0
        p.type = ProgramItem.WAIT_UNTIL_USER_FINISHES
        pb.items.append(deepcopy(p))

        p = ProgramItem()
        p.id = 8
        p.on_success = 9
        p.on_failure = 0
        p.type = ProgramItem.PICK_FROM_POLYGON
        p.object.append("profile")
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
        p.type = ProgramItem.PLACE_TO_POSE
        p.ref_id.append(8)
        pp = PoseStamped()
        pp.header.frame_id = "marker"
        pp.pose.position.x = 0.75
        pp.pose.position.y = 0.5
        p.pose.append(pp)
        pb.items.append(deepcopy(p))

    def test_alt_prog_item(self):

        prog = Program()
        prog.header.id = 123

        pb = ProgramBlock()
        pb.id = 10
        prog.blocks.append(pb)

        pi = ProgramItem()
        pi.id = 1
        pi.type = ProgramItem.PICK_FROM_FEEDER
        pi.object.append("obj1")
        pi.pose.append(PoseStamped())
        pi.on_success = 2
        pb.items.append(pi)

        pi = ProgramItem()
        pi.id = 2
        pi.type = ProgramItem.PLACE_TO_GRID
        pi.polygon.append(PolygonStamped())
        pi.ref_id.append(1)

        for i in range(0, 3):
            pi.pose.append(PoseStamped())

        pi.on_success = 1
        pi.on_failure = 3
        pb.items.append(pi)

        pi = ProgramItem()
        pi.id = 3
        pi.type = ProgramItem.DRILL_POINTS
        pi.ref_id.append(2)
        pi.pose.append(PoseStamped())
        pi.on_success = 3
        pi.on_failure = 1
        pb.items.append(pi)

        res = self.ph.load(prog)
        self.assertEquals(res, True, "alt program")

        ret = self.ph.get_object(10, 3)

        self.assertEquals(ret[0][0], "obj1", "ref_id object")
        self.assertEquals(ret[1], 1, "ref_id object - source item id")

        ret = self.ph.get_polygon(10, 3)

        self.assertEquals(len(ret[0]), 1, "ref_id polygon")
        self.assertEquals(ret[1], 2, "ref_id polygon - source item id")

        self.assertRaises(ValueError, self.ph.get_polygon, 10, 1)

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

        (block_id, item_id) = self.ph.get_id_on_success(1, 2)
        self.assertEquals(block_id, 1, "on_success - block_id")
        self.assertEquals(item_id, 3, "on_success - item_id")

    def test_on_failure(self):

        res = self.ph.load(self.prog)
        self.assertEquals(res, True, "on_failure")

        (block_id, item_id) = self.ph.get_id_on_failure(1, 4)
        self.assertEquals(block_id, 0, "on_failure - block_id")
        self.assertEquals(item_id, 0, "on_failure - item_id")

    def test_get_item_msg(self):

        res = self.ph.load(self.prog)
        self.assertEquals(res, True, "get_item_msg")

        msg = self.ph.get_item_msg(1, 4)
        self.assertEquals(isinstance(msg, ProgramItem), True, "get_item_msg - type")
        self.assertEquals(msg.id, 4, "get_item_msg - id")

    def test_get_block_msg(self):

        res = self.ph.load(self.prog)
        self.assertEquals(res, True, "get_block_msg")

        msg = self.ph.get_block_msg(1)
        self.assertEquals(isinstance(msg, ProgramBlock), True, "get_block_msg - type")
        self.assertEquals(msg.id, 1, "get_block_msg - id")

    def test_get_first_block_id(self):

        res = self.ph.load(self.prog)
        self.assertEquals(res, True, "get_first_block_id")

        block_id = self.ph.get_first_block_id()
        self.assertEquals(block_id, self.prog.blocks[0].id, "get_first_block_id")

    def test_get_first_item_id(self):

        res = self.ph.load(self.prog)
        self.assertEquals(res, True, "get_first_block_id")

        (block_id, item_id) = self.ph.get_first_item_id()
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

    def test_invalid_ref_id(self):

        prog = deepcopy(self.prog)
        prog.blocks[0].items[0].ref_id.append(6587)
        res = self.ph.load(prog)
        self.assertEquals(res, False, "test_invalid_ref_id")


if __name__ == '__main__':

    rospy.init_node('test_node')
    rostest.run('art_utils', 'test_program_helper', TestProgramHelper, sys.argv)
