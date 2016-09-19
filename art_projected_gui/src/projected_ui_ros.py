#!/usr/bin/env python

from art_msgs.msg import InstancesArray


class ProjectedUIRos(ProjectedUI):
    
    def __init__(self, x,  y,  width,  height):
        
        super(ProjectedUIRos,  self).__init__(x,  y,  width,  height)
        
        self.obj_sub = rospy.Subscriber('/art/object_detector/object_filtered', InstancesArray, self.object_cb, queue_size=1)
        
        QtCore.QObject.connect(self, QtCore.SIGNAL('objects'), self.object_cb_evt)
        
    def object_cb(self,  msg):
        
        self.emit(QtCore.SIGNAL('objects'),  msg)
        
    def object_cb_evt(self,  msg):
        
        for obj_id in msg.lost_objects:
            
            try:
                self.scene.remove(self.objects[obj_id])
                del self.objects[obj_id]
            except KeyError:
                pass
        
        for obj in msg.instances:
            
            if obj.object_id not in self.viz_objects:
                
                self.add_object(obj.object_id,  obj.object_type,  obj.pose.position.x,  obj.pose.position.y)
                
            else:
                
                self.objects[obj.object_id].set_pos(obj.pose.position.x,  obj.pose.position.y)
