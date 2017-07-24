#!/usr/bin/env python

import sys
import rospy
from art_msgs.msg import Program, ProgramBlock, ProgramItem, ObjectType
from art_msgs.srv import storeProgram, getObjectType, storeObjectType
from shape_msgs.msg import SolidPrimitive
from geometry_msgs.msg import PoseStamped, PolygonStamped, Point32
from copy import deepcopy


def store_object_type(ot):

    rospy.wait_for_service('/art/db/object_type/store')

    try:
        store_object_srv = rospy.ServiceProxy(
            '/art/db/object_type/store', storeObjectType)
        resp = store_object_srv(ot)
    except rospy.ServiceException as e:
        print "Service call failed: " + str(e)
        return


def store_program(prog):

    rospy.wait_for_service('/art/db/program/store')

    try:
        store_program_srv = rospy.ServiceProxy(
            '/art/db/program/store', storeProgram)
        resp = store_program_srv(program=prog)
        if not resp.success:
            print "Failed to save program ID: " + str(prog.header.id) + ", error: " + resp.error
    except rospy.ServiceException as e:
        print "Service call failed: " + str(e)
        return


def main(args):

    global p

    rospy.init_node('art_db_service_tester', anonymous=True)

    # -------------------------------------------------------------------------------------------
    # PROGRAMS
    # -------------------------------------------------------------------------------------------
    prog = Program()
    prog.header.id = 0
    prog.header.name = "Advanced pick&place"

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
    p.type = ProgramItem.PLACE_TO_POSE
    p.ref_id.append(8)
    pp = PoseStamped()
    pp.header.frame_id = "marker"
    pp.pose.position.x = 0.75
    pp.pose.position.y = 0.5
    p.pose.append(pp)
    pb.items.append(deepcopy(p))

    store_program(prog)

    # -------------------------------------------------------------------------------------------
    prog = Program()
    prog.header.id = 3
    prog.header.name = "Basic pick&place"

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
    p.type = ProgramItem.PICK_OBJECT_ID
    p.object.append("")
    pf = PoseStamped()
    pf.header.frame_id = "marker"
    p.pose.append(pf)
    pb.items.append(deepcopy(p))

    p = ProgramItem()
    p.id = 4
    p.on_success = 3
    p.on_failure = 0
    p.type = ProgramItem.PLACE_TO_POSE
    p.ref_id.append(3)
    pp = PoseStamped()
    pp.header.frame_id = "marker"
    p.pose.append(pp)
    pb.items.append(deepcopy(p))

    store_program(prog)

    # -------------------------------------------------------------------------------------------
    prog = Program()
    prog.header.id = 2
    prog.header.name = "Classic pick&place"

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
    p.type = ProgramItem.PICK_FROM_POLYGON
    p.object.append("")
    pp = PolygonStamped()
    pp.header.frame_id = "marker"
    p.polygon.append(pp)
    pb.items.append(deepcopy(p))

    p = ProgramItem()
    p.id = 4
    p.on_success = 5
    p.on_failure = 0
    p.type = ProgramItem.PLACE_TO_POSE
    p.ref_id.append(3)
    pp = PoseStamped()
    pp.header.frame_id = "marker"
    p.pose.append(pp)
    pb.items.append(deepcopy(p))

    p = ProgramItem()
    p.id = 5
    p.on_success = 6
    p.on_failure = 0
    p.type = ProgramItem.GET_READY
    pb.items.append(deepcopy(p))

    p = ProgramItem()
    p.id = 6
    p.on_success = 7
    p.on_failure = 0
    p.type = ProgramItem.WAIT_UNTIL_USER_FINISHES
    pb.items.append(deepcopy(p))

    p = ProgramItem()
    p.id = 7
    p.on_success = 8
    p.on_failure = 0
    p.type = ProgramItem.PICK_FROM_POLYGON
    p.object.append("")
    p.object.append("profile_20_60")
    pp = PolygonStamped()
    pp.header.frame_id = "marker"
    p.polygon.append(pp)
    pb.items.append(deepcopy(p))

    p = ProgramItem()
    p.id = 8
    p.on_success = 1
    p.on_failure = 0
    p.type = ProgramItem.PLACE_TO_POSE
    p.ref_id.append(7)
    pp = PoseStamped()
    pp.header.frame_id = "marker"
    p.pose.append(pp)
    pb.items.append(deepcopy(p))

    store_program(prog)

    # -------------------------------------------------------------------------------------------
    prog = Program()
    prog.header.id = 3
    prog.header.name = "Welding seam"

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
    p.type = ProgramItem.GET_READY
    pb.items.append(deepcopy(p))

    p = ProgramItem()
    p.id = 2
    p.on_success = 2
    p.on_failure = 0
    p.type = ProgramItem.WELDING_SEAM
    p.object.append("")
    pf = PoseStamped()
    pf.header.frame_id = "marker"
    pf.pose.position.x = 0.4
    pf.pose.position.y = 0.4
    pf.pose.position.z = 0.30
    pf.pose.orientation.x = -0.707
    pf.pose.orientation.y = 0
    pf.pose.orientation.z = 0.707
    pf.pose.orientation.w = 0
    p.pose.append(deepcopy(pf))
    pf.pose.position.x = 0.5
    p.pose.append(deepcopy(pf))
    pf.pose.position.y = 0.2
    p.pose.append(deepcopy(pf))
    pf.pose.position.x = 0.4
    p.pose.append(deepcopy(pf))
    pb.items.append(deepcopy(p))

    store_program(prog)

    # -------------------------------------------------------------------------------------------
    prog = Program()
    prog.header.id = 4
    prog.header.name = "Welding points"

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
    p.type = ProgramItem.GET_READY
    pb.items.append(deepcopy(p))

    p = ProgramItem()
    p.id = 2
    p.on_success = 2
    p.on_failure = 0
    p.type = ProgramItem.WELDING_POINTS
    p.object.append("")
    pf = PoseStamped()
    pf.header.frame_id = "marker"
    pf.pose.position.x = 0.4
    pf.pose.position.y = 0.4
    pf.pose.position.z = 0.30
    pf.pose.orientation.x = -0.707
    pf.pose.orientation.y = 0
    pf.pose.orientation.z = 0.707
    pf.pose.orientation.w = 0
    p.pose.append(deepcopy(pf))
    pf.pose.position.x = 0.5
    p.pose.append(deepcopy(pf))
    pf.pose.position.y = 0.2
    p.pose.append(deepcopy(pf))
    pf.pose.position.x = 0.4
    p.pose.append(deepcopy(pf))
    pb.items.append(deepcopy(p))

    store_program(prog)

    # -------------------------------------------------------------------------------------------
    prog = Program()
    prog.header.id = 5
    prog.header.name = "Drill points"

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
    p.type = ProgramItem.GET_READY
    pb.items.append(deepcopy(p))

    p = ProgramItem()
    p.id = 2
    p.on_success = 2
    p.on_failure = 0
    p.type = ProgramItem.DRILL_POINTS
    p.object.append("")
    pf = PoseStamped()
    pf.header.frame_id = "marker"
    pf.pose.position.x = 0.4
    pf.pose.position.y = 0.4
    pf.pose.position.z = 0.30
    pf.pose.orientation.x = -0.707
    pf.pose.orientation.y = 0
    pf.pose.orientation.z = 0.707
    pf.pose.orientation.w = 0
    p.pose.append(deepcopy(pf))
    pf.pose.position.x = 0.5
    p.pose.append(deepcopy(pf))
    pf.pose.position.y = 0.2
    p.pose.append(deepcopy(pf))
    pf.pose.position.x = 0.4
    p.pose.append(deepcopy(pf))

    dp = PolygonStamped()
    dp.header.frame_id = "marker"
    p.polygon.append(dp)

    pb.items.append(deepcopy(p))

    store_program(prog)

    # -------------------------------------------------------------------------------------------
    prog = Program()
    prog.header.id = 9
    prog.header.name = "Place to grid"

    pb = ProgramBlock()
    pb.id = 1  # can't be zero
    pb.name = "First Block"
    pb.on_success = 1
    pb.on_failure = 0
    prog.blocks.append(pb)

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
    p.type = ProgramItem.PICK_FROM_POLYGON
    p.object.append("")
    p.object.append("profile_20_60")
    pp = PolygonStamped()
    pp.header.frame_id = "marker"
    p.polygon.append(pp)
    pb.items.append(deepcopy(p))

    p = ProgramItem()
    p.id = 4
    p.on_success = 3
    p.on_failure = 0
    p.type = ProgramItem.PLACE_TO_GRID
    p.object.append("")
    pp = PolygonStamped()
    p.polygon.append(pp)
    p.ref_id.append(3)
    pb.items.append(deepcopy(p))

    store_program(prog)

    # -------------------------------------------------------------------------------------------
    # OBJECT TYPES
    # -------------------------------------------------------------------------------------------

    ot = ObjectType()
    ot.name = "profile_20_60"
    ot.bbox.type = SolidPrimitive.BOX
    ot.bbox.dimensions.append(0.04)
    ot.bbox.dimensions.append(0.04)
    ot.bbox.dimensions.append(0.13)

    store_object_type(ot)

    ot.name = "profile_20_80"

    store_object_type(ot)


if __name__ == '__main__':
    try:
        main(sys.argv)
    except KeyboardInterrupt:
        print("Shutting down")
