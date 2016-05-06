#!/usr/bin/env python

from art_msgs.msg import ObjInstance, InstancesArray
from art_msgs.srv import getProgram,  getProgramResponse,  storeProgram,  storeProgramResponse,  getObject,  getObjectResponse,  storeObject,  storeObjectResponse
import sys
import rospy
import dataset
import rospkg
import message_converter
import json
import sqlite3

class ArtDB:
    
    def __init__(self):
        
        rospack = rospkg.RosPack()
        self.db_path = rospack.get_path('art_db') + '/art.db' # TODO where to store?
        
        self.srv_get_program = rospy.Service('/art_db/program/get', getProgram, self.srv_get_program_cb)
        self.srv_store_program = rospy.Service('/art_db/program/store', storeProgram, self.srv_store_program_cb)
        
        self.srv_get_object = rospy.Service('/art_db/object/get', getObject, self.srv_get_object_cb)
        self.srv_store_object = rospy.Service('/art_db/object/store', storeObject, self.srv_store_object_cb)
        
    def srv_get_program_cb(self,  req):
        
        db = dataset.connect('sqlite:////' + self.db_path)
        obj = db['programs'].find_one(program_id=req.id)
        resp = getProgramResponse()
        resp.program = message_converter.convert_dictionary_to_ros_message('art_msgs/Program', json.loads(obj['json']))
        return resp
        
    def srv_store_program_cb(self,  req):
        
        db = dataset.connect('sqlite:////' + self.db_path)
        programs = db['programs']
        prog_json = json.dumps(message_converter.convert_ros_message_to_dictionary(req.program))
        resp = storeProgramResponse()
        resp.success = True
        programs.insert(dict(program_id=req.program.id,  name=req.program.name,  json=prog_json))
        return resp
        
    def srv_get_object_cb(self,  req):
        
        db = dataset.connect('sqlite:////' + self.db_path)
        obj = db['objects'].find_one(obj_id=req.obj_id)
        resp = getObjectResponse()
        resp.name = obj['name']
        resp.model_url = obj['model_url']
        resp.type = obj['type']
        print obj['bb']
        print  json.loads(obj['bb'])
        resp.bbox = message_converter.convert_dictionary_to_ros_message('shape_msgs/SolidPrimitive', json.loads(obj['bb']))
        return resp
        
    def srv_store_object_cb(self,  req):
        
        db = dataset.connect('sqlite:////' + self.db_path)
        objects = db['objects']
        bb_json = json.dumps(message_converter.convert_ros_message_to_dictionary(req.bbox))
        print bb_json
        objects.insert(dict(name=req.name, model_url=req.model_url, obj_id=req.obj_id,  type=req.type,  bb=bb_json))
        resp =storeObjectResponse()
        resp.success = True
        return resp
        
def main(args):
    
    rospy.init_node('art_db', anonymous=True)
    db = ArtDB()
    rospy.spin()
    
if __name__ == '__main__':
    try:
        main(sys.argv)
    except KeyboardInterrupt:
        print("Shutting down")
        
        
