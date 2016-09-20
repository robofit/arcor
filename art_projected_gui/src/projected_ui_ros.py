#!/usr/bin/env python

from projected_ui import ProjectedUI
from PyQt4 import QtGui, QtCore
import rospy
from art_msgs.msg import InstancesArray

class ProjectedUIRos(ProjectedUI):
    
    def __init__(self, x,  y,  width,  height):
        
        super(ProjectedUIRos,  self).__init__(x,  y,  width,  height)
        
        self.obj_sub = rospy.Subscriber('/art/object_detector/object_filtered', InstancesArray, self.object_cb, queue_size=1)
        self.user_status_sub = rospy.Subscriber('/art/user/status',  UserStatus,  self.user_status_cb,  queue_size=1)
        
        QtCore.QObject.connect(self, QtCore.SIGNAL('objects'), self.object_cb_evt)
        QtCore.QObject.connect(self, QtCore.SIGNAL('user_status'), self.user_status_cb_evt)
        
        self.user_status = None
        
    def object_cb(self,  msg):
        
        self.emit(QtCore.SIGNAL('objects'),  msg)
        
    def object_cb_evt(self,  msg):
        
        for obj_id in msg.lost_objects:
            
            self.remove_object(obj_id)
        
        for obj in msg.instances:
            
            if obj.object_id not in self.objects:
                
                self.add_object(obj.object_id,  obj.object_type,  obj.pose.position.x,  obj.pose.position.y)
                
            else:
                
                self.objects[obj.object_id].set_pos(obj.pose.position.x,  obj.pose.position.y)
                
    def user_status_cb(self,  msg):
        
        self.emit(QtCore.SIGNAL('user_status'),  msg)
        
    def user_status_cb_evt(self,  msg):
        
        # TODO
        self.user_status = msg
