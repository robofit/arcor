#!/usr/bin/env python

import sys
import rospy
from art_msgs.msg import ProgramItem, ObjectType
from shape_msgs.msg import SolidPrimitive
from geometry_msgs.msg import PoseStamped, PolygonStamped


def feeder_item(it_id, on_success=None, on_failure=0, obj_type="", ref_id=[]):

    p = item(it_id, ProgramItem.PICK_FROM_FEEDER, on_success, on_failure, ref_id=ref_id)

    if len(ref_id) == 0:

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
    # p.object.append("")
    return p


def grid_item(it_id, ref_id, on_success=None, on_failure=0, objects=2):

    p = item(it_id, ProgramItem.PLACE_TO_GRID, on_success, on_failure, ref_id=ref_id)
    pp = PolygonStamped()
    pp.header.frame_id = "marker"
    p.polygon.append(pp)
    ps = PoseStamped()
    ps.header.frame_id = "marker"
    for i in range(0, objects):
        p.pose.append(ps)
    return p


def drill_item(it_id, ref_id=[], on_success=None, on_failure=0, holes=2, obj_type=[]):

    p = item(it_id, ProgramItem.DRILL_POINTS, on_success, on_failure, ref_id=ref_id)

    if len(ref_id) == 0:

        pp = PolygonStamped()
        pp.header.frame_id = "marker"
        p.polygon.append(pp)

    for obj in obj_type:

        p.object.append(obj)

    ps = PoseStamped()  # pose is relative to the selected object

    for i in range(0, holes):
        p.pose.append(ps)
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
