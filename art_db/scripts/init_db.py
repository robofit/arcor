#!/usr/bin/env python

import sys
import rospy
from art_msgs.msg import Program,  ProgramBlock, ProgramItem,  ObjectType
from art_msgs.srv import getProgram,  storeProgram,   getObjectType,  storeObjectType
from shape_msgs.msg import SolidPrimitive


def main(args):

    global p

    rospy.init_node('art_db_service_tester', anonymous=True)

    p = Program()
    p.id = 0
    p.name = "Basic pick&place"

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

    rospy.wait_for_service('/art/db/program/store')

    try:
        store_program_srv = rospy.ServiceProxy('/art/db/program/store', storeProgram)
        resp = store_program_srv(program=p)
    except rospy.ServiceException, e:
        print "Service call failed: " + str(e)
        return

    rospy.wait_for_service('/art/db/program/get')

    try:
        get_program_srv = rospy.ServiceProxy('/art/db/program/get', getProgram)
        resp = get_program_srv(id=0)
        print resp
    except rospy.ServiceException, e:
        print "Service call failed: " + str(e)
        return

    rospy.wait_for_service('/art/db/object_type/store')

    ot = ObjectType()
    ot.name = "profile_20_60"
    ot.bbox.type = SolidPrimitive.BOX
    ot.bbox.dimensions.append(0.05)
    ot.bbox.dimensions.append(0.05)
    ot.bbox.dimensions.append(0.16)

    try:
        store_object_srv = rospy.ServiceProxy('/art/db/object_type/store', storeObjectType)
        resp = store_object_srv(ot)
    except rospy.ServiceException, e:
        print "Service call failed: " + str(e)
        return

    ot.name = "profile_20_80"

    try:
        store_object_srv = rospy.ServiceProxy('/art/db/object_type/store', storeObjectType)
        resp = store_object_srv(ot)
    except rospy.ServiceException, e:
        print "Service call failed: " + str(e)
        return

    rospy.wait_for_service('/art/db/object_type/get')

    try:
        get_object_srv = rospy.ServiceProxy('/art/db/object_type/get', getObjectType)
        resp = get_object_srv(name="profile_20_60")
        print resp
        resp = get_object_srv(name="profile_20_80")
        print resp
    except rospy.ServiceException, e:
        print "Service call failed: " + str(e)
        return

if __name__ == '__main__':
    try:
        main(sys.argv)
    except KeyboardInterrupt:
        print("Shutting down")
