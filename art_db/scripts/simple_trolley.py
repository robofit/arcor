#!/usr/bin/env python

import sys
import rospy
from art_msgs.msg import Program, ProgramBlock, ProgramItem, ObjectType
from art_msgs.srv import storeProgram, getObjectType, storeObjectType
from shape_msgs.msg import SolidPrimitive
from geometry_msgs.msg import PoseStamped, PolygonStamped, Point32
from copy import deepcopy


def store_program(prog):

    rospy.wait_for_service('/art/db/program/store')

    try:
        store_program_srv = rospy.ServiceProxy(
            '/art/db/program/store', storeProgram)
        resp = store_program_srv(program=prog)
    except rospy.ServiceException as e:
        print "Service call failed: " + str(e)
        return


def item(it_id, it_type, on_success=it_id + 1, on_failure=0, ref_id=[]):

    p = ProgramItem()
    p.id = it_id
    p.type = it_type
    p.on_success = on_success
    p.on_failure = on_failure
    p.ref_id = ref_id

    return p


def main(args):

    rospy.init_node('art_db_service_tester', anonymous=True)

    # -------------------------------------------------------------------------------------------
    # Simplified trolley assembly program
    # -------------------------------------------------------------------------------------------
    prog = Program()
    prog.header.id = 20
    prog.header.name = "Simple Trolley Assembly"

    # only for first iteration
    pb = ProgramBlock()
    pb.id = 1
    pb.name = "Initial block"
    pb.on_success = 2
    pb.on_failure = 0
    prog.blocks.append(pb)

    pb.items.append(item(1, ProgramItem.GET_READY, 2))
    pb.items.append(item(2, ProgramItem.WAIT_FOR_USER, 0))

    # --- left side of the trolley ------------------------------------------------------
    pb = ProgramBlock()
    pb.id = 2
    pb.name = "Left side"
    pb.on_success = 3
    pb.on_failure = 0
    prog.blocks.append(pb)

    pb.items.append(item(2, ProgramItem.WAIT_UNTIL_USER_FINISHES))

    # each side consists of four profiles
    pb.items.append(item(3, ProgramItem.PICK_FROM_FEEDER))
    pb.items.append(item(4, ProgramItem.PLACE_TO_POSE, ref_id=[3]))

    pb.items.append(item(5, ProgramItem.PICK_FROM_FEEDER))
    pb.items.append(item(6, ProgramItem.PLACE_TO_POSE, ref_id=[4]))

    pb.items.append(item(7, ProgramItem.PICK_FROM_FEEDER))
    pb.items.append(item(8, ProgramItem.PLACE_TO_POSE, ref_id=[5]))

    pb.items.append(item(9, ProgramItem.PICK_FROM_FEEDER))
    pb.items.append(item(10, ProgramItem.PLACE_TO_POSE, ref_id=[6]))

    # after p&p, let's drill holes
    pb.items.append(item(11, ProgramItem.DRILL_POINTS))
    pb.items.append(item(12, ProgramItem.DRILL_POINTS))
    pb.items.append(item(13, ProgramItem.DRILL_POINTS))
    pb.items.append(item(14, ProgramItem.DRILL_POINTS))

    # --- right side of the trolley ------------------------------------------------------
    pb = deepcopy(pb)
    pb.id = 3
    pb.name = "Right side"
    pb.on_success = 4
    pb.on_failure = 0
    prog.blocks.append(pb)

    # --- connecting profiles ------------------------------------------------------------
    pb = ProgramBlock()
    pb.id = 2
    pb.name = "Connecting profiles"
    pb.on_success = 0
    pb.on_failure = 0
    prog.blocks.append(pb)


if __name__ == '__main__':
    try:
        main(sys.argv)
    except KeyboardInterrupt:
        print("Shutting down")
