#!/usr/bin/env python

import rospy
import rospkg
import sys
import signal

import cv2
from std_msgs.msg import String
from PyQt4 import QtGui, QtCore, QtOpenGL
from art_msgs.msg import InstancesArray
from geometry_msgs.msg import PoseStamped
from std_srvs.srv import Empty, EmptyResponse

from helper_objects import scene_place,  scene_object,  pointing_point

# 12.5 x 10

# TODO notifications ?
# TODO draw bottom side of bounding box
# TODO fix position of object_id

def sigint_handler(*args):
    """Handler for the SIGINT signal."""
    sys.stderr.write('\r')
    QtGui.QApplication.quit()


class simple_gui(QtGui.QWidget):

    def __init__(self):
       
       super(simple_gui, self).__init__()
       
       self.inited = False
       
       rospack = rospkg.RosPack()
       self.img_path = rospack.get_path('art_simple_gui') + '/imgs'
       
       self.obj_sub = rospy.Subscriber('/art_object_detector/object_filtered', InstancesArray, self.object_cb)
       self.point_left_sub = rospy.Subscriber('/pointing_left', PoseStamped, self.pointing_point_left_cb)
       self.point_right_sub = rospy.Subscriber('/pointing_right', PoseStamped, self.pointing_point_right_cb)
       
       self.selected_object_pub = rospy.Publisher("/art_simple_gui/selected_object", String, queue_size=10)
       self.selected_place_pub = rospy.Publisher("/art_simple_gui/selected_place", PoseStamped, queue_size=10)
       
       self.srv_show_marker = rospy.Service('/art_simple_gui/show_marker', Empty, self.show_marker)
       self.srv_hide_marker = rospy.Service('/art_simple_gui/hide_marker', Empty, self.hide_marker)
       self.srv_clear_all = rospy.Service('/art_simple_gui/clear_all', Empty, self.clear_all) # clear all selections etc.
       
       self.marker_box_size = rospy.get_param("/art_params/marker_box_size", 0.0685)
       
       self.objects = None
       self.viz_objects = {}
       self.viz_places = []
       
       self.initUI()
       
       self.pointing_left = pointing_point("left", self.scene)
       self.pointing_right = pointing_point("right", self.scene)
       
       QtCore.QObject.connect(self, QtCore.SIGNAL('objects()'), self.objects_evt)
       QtCore.QObject.connect(self, QtCore.SIGNAL('pointing_point()'), self.pointing_point_evt)
       QtCore.QObject.connect(self, QtCore.SIGNAL('clear_all()'), self.clear_all_evt)
       QtCore.QObject.connect(self, QtCore.SIGNAL('show_marker()'), self.show_marker_evt)
       QtCore.QObject.connect(self, QtCore.SIGNAL('hide_marker()'), self.hide_marker_evt)
       
       #self.timer = QtCore.QTimer()
       #self.timer.start(500)
       #self.timer.timeout.connect(self.timer_evt)
       
       self.inited = True
    
    def show_marker(self, req):
        
        self.emit(QtCore.SIGNAL('show_marker()'))
        return EmptyResponse()
        
    def show_marker_evt(self):
        
        self.marker.show()
        self.update()
        
    def hide_marker(self, req):
        
        self.emit(QtCore.SIGNAL('hide_marker()'))
        return EmptyResponse()
    
    def hide_marker_evt(self):
        
        self.marker.hide()
        self.update()
    
    def clear_all_evt(self):
    
        for k,  v in self.viz_objects.iteritems():
            
            v.unselect()
    
        for it in self.viz_places:
            it.remove()
        self.selected_places = []
        
    def clear_all(self, req):
    
        self.emit(QtCore.SIGNAL('clear_all()'))
        return EmptyResponse()
       
    def initUI(self):
       
       self.scene=QtGui.QGraphicsScene(self)
       self.scene.setBackgroundBrush(QtCore.Qt.black)
       self.view = QtGui.QGraphicsView(self.scene, self)
       self.view.setRenderHint(QtGui.QPainter.Antialiasing)
       #self.view.setViewport(QtOpenGL.QGLWidget())
       
       self.pm = QtGui.QPixmap(self.img_path + "/koberec.png")
       self.marker = self.scene.addPixmap(self.pm.scaled(self.size(), QtCore.Qt.KeepAspectRatio))
       self.marker.setZValue(-100)
       self.marker.hide()
       
       self.resizeEvent = self.on_resize
    
    def timer_evt(self):
    
        pass
    
    def on_resize(self, event):
    
        print "on_resize"
        self.view.setFixedSize(self.width(), self.height())
        self.view.setSceneRect(QtCore.QRectF(0, 0, self.width(), self.height()))
        self.view.setHorizontalScrollBarPolicy(QtCore.Qt.ScrollBarAlwaysOff);
        self.view.setVerticalScrollBarPolicy(QtCore.Qt.ScrollBarAlwaysOff);
        self.marker.setPixmap(self.pm.scaled(self.size(), QtCore.Qt.KeepAspectRatio))
      
    def object_cb(self, msg):
        
        if not self.inited: return
   
        self.objects = msg.instances
        self.emit(QtCore.SIGNAL('objects()'))
    
    def pointing_point_evt(self):
        
        points = [self.pointing_left,  self.pointing_right]
        
        for pt in points:
    
            if not pt.is_active(): continue
                
            for k, v in self.viz_objects.iteritems():
                
                v.pointing(pt.viz)
        
            if pt.viz is None: continue
            
            items = pt.viz.collidingItems()
            
            pointed_place = pt.get_pointed_place()
            if len(items) == 0 and pointed_place is not None:

                rospy.loginfo("New place selected")
                self.viz_places.append(scene_place(self.scene,  pointed_place,  self.marker_box_size,  self.selected_place_pub))
        
        self.update()        
        
    def objects_evt(self):
    
       current_objects = {}
    
       for obj in self.objects:
       
               current_objects[obj.object_id] = None
       
               (px, py) = self.get_px(obj.pose)
  
               if obj.object_id not in self.viz_objects:
                    
                    sobj = scene_object(self.scene,  obj.object_id,  (px,  py),  self.selected_object_pub)
                    self.viz_objects[obj.object_id] = sobj
                    
               else:
                   
                   self.viz_objects[obj.object_id].set_pos((px,  py))
       
       to_delete = []            
       for k, v in self.viz_objects.iteritems():
       
           if k not in current_objects:
           
               to_delete.append(k)
               v.remove()
               
       for d in to_delete:
       
           del self.viz_objects[d]
           
       self.update()
    
    def get_px(self, pose):
    
       px = self.width() - int((pose.position.x / self.marker_box_size) * self.height()/10.0)
       py = int((pose.position.y / self.marker_box_size) * self.height()/10.0)
       
       return (px, py)
     
    def pointing_point_left_cb(self, msg):
        
        if not self.inited: return
       
        (px, py) = self.get_px(msg.pose)
       
        if (px in range(0, self.width()) and py in range(0, self.height())):
           
           self.pointing_left.set_pos((px,  py))
           #print("px: " + str(px) + " py: " + str(py))
           self.emit(QtCore.SIGNAL('pointing_point()'))
           
    def pointing_point_right_cb(self, msg):
        
        if not self.inited: return
       
        (px, py) = self.get_px(msg.pose)
       
        if (px in range(0, self.width()) and py in range(0, self.height())):
       
           self.pointing_right.set_pos((px,  py))
           #print("px: " + str(px) + " py: " + str(py))
           self.emit(QtCore.SIGNAL('pointing_point()'))
       
def main(args):
    
    rospy.init_node('simple_gui', anonymous=True)
    
    signal.signal(signal.SIGINT, sigint_handler)

    app = QtGui.QApplication(sys.argv)
    window = simple_gui()
    
    desktop = QtGui.QDesktopWidget()
    geometry = desktop.screenGeometry(1) # 1
    window.move(geometry.left(), geometry.top())
    window.resize(geometry.width(), geometry.height())
    window.showFullScreen()
    
    timer = QtCore.QTimer()
    timer.start(500)
    timer.timeout.connect(lambda: None)  # Let the interpreter run each 500 ms.

    sys.exit(app.exec_())
    
if __name__ == '__main__':
    try:
        main(sys.argv)
    except KeyboardInterrupt:
        print("Shutting down")
