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
       
       rospack = rospkg.RosPack()
       self.img_path = rospack.get_path('art_simple_gui') + '/imgs'
       
       self.obj_sub = rospy.Subscriber('/art_object_detector/object_filtered', InstancesArray, self.object_cb)
       self.point_sub = rospy.Subscriber('/pointing_point', PoseStamped, self.pointing_point)
       
       self.selected_object_pub = rospy.Publisher("/art_simple_gui/selected_object", String, queue_size=10)
       self.selected_place_pub = rospy.Publisher("/art_simple_gui/selected_place", PoseStamped, queue_size=10)
       
       self.srv_show_marker = rospy.Service('/art_simple_gui/show_marker', Empty, self.show_marker)
       self.srv_hide_marker = rospy.Service('/art_simple_gui/hide_marker', Empty, self.hide_marker)
       self.srv_clear_all = rospy.Service('/art_simple_gui/clear_all', Empty, self.clear_all) # clear all selections etc.
       
       self.marker_box_size = rospy.get_param("/art_params/marker_box_size", 0.075)
       
       self.objects = None
       self.objects_ellipses = {}
       self.objects_labels = {}

       self.preselected_object = None # pre-select (gray)
       self.selected_at = None
       self.selection_conf = False
       self.selected_objects = []
       
       self.px = None
       self.py = None
       self.pointing_point = None
       
       self.last_px = None
       self.last_py = None
       
       self.place_selection_time = None
       self.selected_places = []
       
       self.initUI()
       
       QtCore.QObject.connect(self, QtCore.SIGNAL('objects()'), self.objects_evt)
       QtCore.QObject.connect(self, QtCore.SIGNAL('pointing_point()'), self.pointing_point_evt)
       QtCore.QObject.connect(self, QtCore.SIGNAL('clear_all()'), self.clear_all_evt)
       
       self.timer = QtCore.QTimer()
       self.timer.start(100)
       self.timer.timeout.connect(self.timer_evt)
    
    def show_marker(self, req):
        # TODO use signal
        self.marker.show()
        return EmptyResponse()
        
    def hide_marker(self, req):
        # TODO use signal
        self.marker.hide()
        return EmptyResponse()
        
    def clear_all_evt(self):
    
        if self.preselected_object is not None: self.scene.removeItem(self.preselected_object)
        self.preselected_object = None
        for it in self.selected_objects:
            self.scene.removeItem(it)
        self.selected_objects = []
        for it in self.selected_places:
            self.scene.removeItem(it)
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
    
        # TODO check for missing pointing point and do something about it
    
        if self.preselected_object is not None:
        
            if (rospy.Time.now() - self.selected_at > rospy.Duration(2)):
            
                for k,v in self.objects_ellipses.iteritems():
                    if v is self.preselected_object.parentItem():
                        self.selected_object_pub.publish(k)
                        
                        sel = self.scene.addEllipse(0, 0, 180, 180, QtCore.Qt.green, QtCore.Qt.green)
                        sel.setParentItem(self.preselected_object.parentItem())
                        sel.setPos(150/2 - 180/2, 150/2 - 180/2)
                        sel.setFlag(QtGui.QGraphicsItem.ItemStacksBehindParent)
                        
                        self.selected_objects.append(sel)
                        
                        self.scene.removeItem(self.preselected_object)
                        self.preselected_object = None
                        self.selected_at = None
                        
                        break
    
    def on_resize(self, event):
    
        print "on_resize"
        self.view.setFixedSize(self.width(), self.height())
        self.view.setSceneRect(QtCore.QRectF(0, 0, self.width(), self.height()))
        self.view.setHorizontalScrollBarPolicy(QtCore.Qt.ScrollBarAlwaysOff);
        self.view.setVerticalScrollBarPolicy(QtCore.Qt.ScrollBarAlwaysOff);
        self.marker.setPixmap(self.pm.scaled(self.size(), QtCore.Qt.KeepAspectRatio))
      
    def object_cb(self, msg):
   
       self.objects = msg.instances
       self.emit(QtCore.SIGNAL('objects()'))
    
    def pointing_point_evt(self):
    
        if self.pointing_point is None:
        
            self.pointing_point = self.scene.addEllipse(0, 0, 50, 50, QtCore.Qt.blue, QtCore.Qt.blue)
            self.pointing_point.setPos(self.px - 25, self.py - 25)
            self.pointing_point.setZValue(100)
            
        else:
        
            self.pointing_point.setPos(self.px, self.py)
            
        items = self.pointing_point.collidingItems()
        
        
        if len(items) == 0:
        
            # clear preselect
            if self.preselected_object is not None:
                rospy.loginfo("Clearing pre-select of the object xyz") # TODO object name
                self.scene.removeItem(self.preselected_object)
                self.preselected_object = None
                self.selected_at = None
        
            # intersection with no object might mean we are selecting a place!
            if self.last_px is None:
            
                self.last_px = self.px
                self.last_py = self.py
                
            else:
            
                #print(abs(self.px - self.last_px))
            
                if abs(self.px - self.last_px) < self.width()*0.01 and abs(self.py - self.last_py) < self.height()*0.01:
                
                    if self.place_selection_time is None:
                    
                        self.place_selection_time = rospy.Time.now()
                        
                    else:
                    
                        if rospy.Time.now() - self.place_selection_time > rospy.Duration(2):
                        
                            if self.scene.itemAt(QtCore.QPointF(self.px, self.py)) not in self.selected_places:
                        
                                rospy.loginfo("New place selected")
                        
                                pl = self.scene.addEllipse(0, 0, 100, 100, QtCore.Qt.cyan, QtCore.Qt.cyan)
                                pl.setPos(self.px - 100/2, self.py - 100/2)
                            
                                self.selected_places.append(pl)
                                
                                self.selected_place_pub.publish(self.get_pose(self.px, self.py))
                                
                            else:
                            
                                rospy.loginfo("Already selected place")
                    
                else:
                
                    self.place_selection_time = None
                    
                self.last_px = self.px
                self.last_py = self.py
        
        already_selected = False
        
        for it in items:
        
            if it not in self.objects_ellipses.values(): continue
        
            for so in self.selected_objects:
            
                if so.parentItem() is it:
                    already_selected = True
                    break
        
            if already_selected: continue
        
            if self.preselected_object is None:
            
                self.preselected_object = self.scene.addEllipse(0, 0, 180, 180, QtCore.Qt.gray, QtCore.Qt.gray)
                self.preselected_object.setParentItem(it)
                self.selected_at = rospy.Time.now()
                
            self.preselected_object.setPos(150/2 - 180/2, 150/2 - 180/2)
            self.preselected_object.setFlag(QtGui.QGraphicsItem.ItemStacksBehindParent)
            break
        
    def objects_evt(self):
    
       current_objects = {}
    
       for obj in self.objects:
       
               current_objects[obj.object_id] = None
       
               (px, py) = self.get_px(obj.pose)
       
               if obj.object_id not in self.objects_ellipses:
           
                   self.objects_ellipses[obj.object_id] = self.scene.addEllipse(0, 0, 150, 150, QtCore.Qt.white, QtCore.Qt.white)
                   self.objects_ellipses[obj.object_id].setPos(px - 150/2, py - 150/2)
                   
                   self.objects_labels[obj.object_id] = self.scene.addText(obj.object_id, QtGui.QFont('Arial', 16))
                   self.objects_labels[obj.object_id].setPos(px + 120, py - 10)
                   self.objects_labels[obj.object_id].rotate(180)
                   self.objects_labels[obj.object_id].setDefaultTextColor(QtCore.Qt.white)
                   
               else:
               
                   self.objects_ellipses[obj.object_id].setPos(px - 150/2, py - 150/2)
                   self.objects_labels[obj.object_id].setPos(px + 120 - 150/2, py - 10 - 150/2)
       
       to_delete = []            
       for k, v in self.objects_ellipses.iteritems():
       
           if k not in current_objects:
           
               to_delete.append(k)
               
       for d in to_delete:
       
           self.scene.removeItem(self.objects_ellipses[d])
           self.scene.removeItem(self.objects_labels[d])
       
           del self.objects_ellipses[d]
           del self.objects_labels[d]
            
    def get_pose(self, px, py):
    
        ps = PoseStamped()
        ps.header.frame_id = "marker"
        ps.header.stamp = rospy.Time.now()
        ps.pose.position.x = (self.width() - px)*self.marker_box_size/(self.height()/10.0)
        ps.pose.position.y = py*self.marker_box_size/(self.height()/10.0)
        ps.pose.position.z = 0.0
        ps.pose.orientation.x = 0.0
        ps.pose.orientation.y = 0.0
        ps.pose.orientation.z = 0.0
        ps.pose.orientation.w = 1.0
        
        return ps
    
    def get_px(self, pose):
    
       px = self.width() - int((pose.position.x / self.marker_box_size) * self.height()/10.0)
       py = int((pose.position.y / self.marker_box_size) * self.height()/10.0)
       
       return (px, py)
     
    def pointing_point(self, msg):
       
       (px, py) = self.get_px(msg.pose)
       
       if (px in range(0, self.width()) and py in range(0, self.height())):
       
           self.px = px
           self.py = py
       
           #print("px: " + str(self.px) + " py: " + str(self.py))
           self.emit(QtCore.SIGNAL('pointing_point()'))
       
def main(args):
    
    rospy.init_node('simple_gui', anonymous=True)
    
    signal.signal(signal.SIGINT, sigint_handler)

    app = QtGui.QApplication(sys.argv)
    window = simple_gui()
    
    desktop = QtGui.QDesktopWidget()
    geometry = desktop.screenGeometry(0) # 1
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
