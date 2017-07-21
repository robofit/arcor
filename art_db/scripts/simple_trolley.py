#!/usr/bin/env python

import sys
import rospy
from art_msgs.msg import Program, ProgramBlock, ProgramItem, ObjectType, KeyValue
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


def store_object_type(ot):

    rospy.wait_for_service('/art/db/object_type/store')

    try:
        store_object_srv = rospy.ServiceProxy(
            '/art/db/object_type/store', storeObjectType)
        resp = store_object_srv(ot)
    except rospy.ServiceException as e:
        print "Service call failed: " + str(e)
        return


def feeder_item(it_id, on_success=None, on_failure=0, obj_type=""):

    p = item(it_id, ProgramItem.PICK_FROM_FEEDER, on_success, on_failure)
    ps = PoseStamped()
    ps.header.frame_id = "marker"
    p.pose.append(ps)
    p.object.append(obj_type)
    return p


def place_item(it_id, ref_id, on_success=None, on_failure=0):

    p = item(it_id, ProgramItem.PLACE_TO_POSE, on_success, on_failure, ref_id=ref_id)
    ps = PoseStamped()
    ps.header.frame_id = "marker"
    p.pose.append(ps)
    p.object.append("")
    return p


def grid_item(it_id, ref_id, on_success=None, on_failure=0):

    p = item(it_id, ProgramItem.PLACE_TO_GRID, on_success, on_failure, ref_id=ref_id)
    p.object.append("")
    pp = PolygonStamped()
    pp.header.frame_id = ""
    p.polygon.append(pp)
    return p


def drill_item(it_id, ref_id, on_success=None, on_failure=0):

    p = item(it_id, ProgramItem.DRILL_POINTS, on_success, on_failure, ref_id=ref_id)
    ps = PoseStamped()  # pose is relative to the selected object
    p.pose.append(ps)
    p.pose.append(ps)
    p.object.append("")
    pp = PolygonStamped()
    pp.header.frame_id = "marker"
    p.polygon.append(pp)
    return p


def wait_item(it_id, on_success=None, on_failure=0):

    p = item(it_id, ProgramItem.WAIT_UNTIL_USER_FINISHES, on_success, on_failure)
    pp = PolygonStamped()
    pp.header.frame_id = "marker"
    p.polygon.append(pp)
    return p


def item(it_id, it_type, on_success=None, on_failure=0, ref_id=[]):

    if on_success is None:
        on_success = it_id + 1

    p = ProgramItem()
    p.id = it_id
    p.type = it_type
    p.on_success = on_success
    p.on_failure = on_failure
    p.ref_id = ref_id

    return p


def obj_type(type_name, bbx, bby, bbz):

    ot = ObjectType()
    ot.name = type_name
    ot.bbox.type = SolidPrimitive.BOX
    ot.bbox.dimensions = [bbx, bby, bbz]
    return ot


def main(args):

    rospy.init_node('art_db_service_tester', anonymous=True)

    # -------------------------------------------------------------------------------------------
    # Simplified trolley assembly: object types
    # -------------------------------------------------------------------------------------------

    store_object_type(obj_type("p40x40x200", 0.04, 0.04, 0.2))
    store_object_type(obj_type("p40x40x400", 0.04, 0.04, 0.4))
    store_object_type(obj_type("p40x40x500", 0.04, 0.04, 0.5))

    # -------------------------------------------------------------------------------------------
    # Simplified trolley assembly: program
    # -------------------------------------------------------------------------------------------

    prog = Program()
    prog.header.id = 20
    prog.header.name = "Simple Trolley Assembly"

    # --- left side of the trolley ------------------------------------------------------
    pb = ProgramBlock()
    pb.id = 2
    pb.name = "Left side"
    pb.on_success = 3
    pb.on_failure = 0
    prog.blocks.append(pb)

    pb.items.append(item(2, ProgramItem.WAIT_UNTIL_USER_FINISHES))

    # each side consists of four profiles (of two types)
    pb.items.append(feeder_item(3, obj_type="p40x40x400"))
    pb.items.append(grid_item(4, on_success=3, on_failure=5, ref_id=[3]))

    pb.items.append(feeder_item(5, obj_type="p40x40x200"))
    pb.items.append(grid_item(6, on_success=5, on_failure=11, ref_id=[5]))

    # after p&p, let's drill holes
    pb.items.append(drill_item(11, on_success=11, on_failure=12, ref_id=[4]))
    pb.items.append(drill_item(12, on_success=12, on_failure=13, ref_id=[6]))

    pb.items.append(item(13, ProgramItem.GET_READY, on_success=0))

    # --- right side of the trolley ------------------------------------------------------
    pb = deepcopy(pb)
    pb.id = 3
    pb.name = "Right side"
    pb.on_success = 4
    pb.on_failure = 0
    prog.blocks.append(pb)

    # --- connecting profiles ------------------------------------------------------------
    pb = ProgramBlock()
    pb.id = 4
    pb.name = "Connecting profiles"
    pb.on_success = 2
    pb.on_failure = 0
    prog.blocks.append(pb)

    pb.items.append(item(2, ProgramItem.WAIT_UNTIL_USER_FINISHES))

    pb.items.append(feeder_item(1, obj_type="p40x40x200"))
    pb.items.append(grid_item(2, on_success=1, on_failure=3, ref_id=[1]))

    pb.items.append(item(3, ProgramItem.GET_READY, on_success=0))

    store_program(prog)


if __name__ == '__main__':
    try:
        main(sys.argv)
    except KeyboardInterrupt:
        print("Shutting down")
