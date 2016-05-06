#!/usr/bin/env python

import sys
import rospy
from art_msgs.msg import Program,  ProgramItem
from art_msgs.srv import getProgram,  storeProgram,   getObject,  storeObject
from shape_msgs.msg import SolidPrimitive

def main(args):
    
    global p

    rospy.init_node('art_db_service_tester', anonymous=True)
    
    p = Program()
    p.id = 0
    p.name = "Basic pick&place"
    
    p0 = ProgramItem()
    p0.id = 0
    p0.on_success = 1
    p0.on_failure = 100
    p0.type = ProgramItem.GET_READY
    p.items.append(p0)
    
    p1 = ProgramItem()
    p1.id = 1
    p1.on_success = 2
    p1.on_failure = 100
    p1.type = ProgramItem.WAIT
    p1.spec = ProgramItem.WAIT_FOR_USER
    p.items.append(p1)
    
    p2 = ProgramItem()
    p2.id = 2
    p2.on_success = 3
    p2.on_failure = 100
    p2.type = ProgramItem.MANIP_PICK_PLACE
    p2.spec = ProgramItem.MANIP_TYPE
    p2.object = "profile"
    # TODO p2.pick_polygon
    p2.place_pose.header.frame_id = "marker"
    p2.place_pose.pose.position.x = 0.75
    p2.place_pose.pose.position.y = 0.5
    p.items.append(p2)
    
    p3 = ProgramItem()
    p3.id = 3
    p3.on_success = 4
    p3.on_failure = 100
    p3.type = ProgramItem.WAIT
    p3.spec = ProgramItem.WAIT_UNTIL_USER_FINISHES
    p.items.append(p3)
    
    p4 = ProgramItem()
    p4.id = 4
    p4.on_success = 0
    p4.on_failure = 100
    p4.type = ProgramItem.MANIP_PICK_PLACE
    p4.spec = ProgramItem.MANIP_TYPE
    p4.object = "profile"
    # TODO p4.pick_polygon?
    p4.pick_pose = p2.place_pose
    p4.place_pose.header.frame_id = "marker"
    p4.place_pose.pose.position.x = 0.25
    p4.place_pose.pose.position.y = 0.5
    p.items.append(p4)
    
    rospy.wait_for_service('/art_db/program/store')
    
    try:
        store_program_srv = rospy.ServiceProxy('/art_db/program/store', storeProgram)
        resp = store_program_srv(program=p)
    except rospy.ServiceException, e:
        print "Service call failed: %s"%e
        return
    
    rospy.wait_for_service('/art_db/program/get')
    
    try:
        get_program_srv = rospy.ServiceProxy('/art_db/program/get', getProgram)
        resp = get_program_srv(id=0)
        print resp
    except rospy.ServiceException, e:
        print "Service call failed: %s"%e
        return
        
    rospy.wait_for_service('/art_db/object/store')
    
    # self.objects_table.insert(dict(name="profile_3", model_url="blablabla", obj_id=16))
    # self.objects_table.insert(dict(name="profile_2", model_url="blablabla", obj_id=16))
    # self.objects_table.update(dict(name="profile_3", model_url="blablabla", obj_id=15), ['name'])
    
    bb = SolidPrimitive()
    bb.type = SolidPrimitive.BOX
    bb.dimensions.append(0.1)
    bb.dimensions.append(0.1)
    bb.dimensions.append(0.1)
    
    try:
        store_object_srv = rospy.ServiceProxy('/art_db/object/store', storeObject)
        resp = store_object_srv(obj_id=3,  name="profile_2",  model_url="",  type="profile",  bbox=bb)
        resp = store_object_srv(obj_id=4,  name="profile_3",  model_url="",  type="profile",  bbox=bb)
    except rospy.ServiceException, e:
        print "Service call failed: %s"%e
        return
        
    rospy.wait_for_service('/art_db/object/get')
    
    try:
        get_object_srv = rospy.ServiceProxy('/art_db/object/get', getObject)
        resp = get_object_srv(obj_id=3)
        print resp
        resp = get_object_srv(obj_id=4)
        print resp
        resp = get_object_srv(obj_id=50)
        print resp
    except rospy.ServiceException, e:
        print "Service call failed: %s"%e
        return
    
    
if __name__ == '__main__':
    try:
        main(sys.argv)
    except KeyboardInterrupt:
        print("Shutting down")
