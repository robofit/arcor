#!/usr/bin/env python

import rospy
import rospkg
import sys

import cv2
from std_msgs.msg import String
from PyQt4 import QtGui, QtCore
from art_msgs.msg import InstancesArray
from geometry_msgs.msg import PoseStamped

# 12.5 x 10

# TODO pub selected object
# TODO notifications
# TODO draw bottom side of bounding box
# TODO fix object_id (rotate)
# TODO empty service -> enable/disable marker (draw over marker when enabled)

class simple_gui(QtGui.QWidget):

    def __init__(self):
       
       super(simple_gui, self).__init__()
       
       rospack = rospkg.RosPack()
       self.img_path = rospack.get_path('art_simple_gui') + '/imgs'
       
       self.obj_sub = rospy.Subscriber('/art_object_detector/object_filtered', InstancesArray, self.object_cb)
       self.point_sub = rospy.Subscriber('/pointing_point', PoseStamped, self.pointing_point)
       
       self.objects = None
       
       self.px = None
       self.py = None
       
       self.initUI()
       
       QtCore.QObject.connect(self, QtCore.SIGNAL('repaint()'), self.repaint_evt)
       
    def initUI(self):

       #self.label = QtGui.QLabel(self)
       #pm = QtGui.QPixmap(self.img_path + "/koberec.png")
       #pm.scaled(self.label.size(), QtCore.Qt.KeepAspectRatio)
       #self.label.setPixmap(pm)
       #self.label.setScaledContents(True)
       
       p = self.palette()
       p.setColor(self.backgroundRole(), QtCore.Qt.black)
       self.setPalette(p)
       
       self.resizeEvent = self.on_resize
    
    def on_resize(self, event):
        print "on_resize"
        #self.label.setFixedWidth(self.width())
        #self.label.setFixedHeight(self.height())
      
    def object_cb(self, msg):
   
       self.objects = msg.instances
       self.emit(QtCore.SIGNAL('repaint()'))
   
    def paintEvent(self, e):

       qp = QtGui.QPainter()
       qp.begin(self)
       #qp.setRenderHint(QPainter.Antialiasing)
       
       if self.objects is not None:
       
           qp.setPen(QtCore.Qt.white)
           qp.setBrush(QtCore.Qt.white)
       
           for obj in self.objects:
           
               (px, py) = self.get_px(obj.pose)
               qp.drawEllipse(px-150/2, py-150/2, 150, 150)
               
               qp.setFont(QtGui.QFont('Arial', 16))
               #qp.rotate(180)
               qp.drawText(QtCore.QPoint(px-150/2, py-150/2 - 90), obj.object_id) 
               #qp.rotate(-180) 
       
       if (self.px is not None and self.py is not None):
       
           qp.setPen(QtCore.Qt.blue)
           qp.setBrush(QtCore.Qt.blue)
           
           qp.drawEllipse(self.px, self.py, 50, 50)
       
       qp.end()
    
    def repaint_evt(self):
    
       self.repaint()
    
    def get_px(self, pose):
    
       px = 1280 - int((pose.position.x / 0.075) * 1024/10.0)
       py = int((pose.position.y / 0.075) * 1024/10.0)
       
       return (px, py)
     
    def pointing_point(self, msg):
       
       (px, py) = self.get_px(msg.pose)
       
       if (px in range(0, 1280) and py in range(0, 1024)):
       
           self.px = px
           self.py = py
       
           #print("px: " + str(self.px) + " py: " + str(self.py))
           self.emit(QtCore.SIGNAL('repaint()'))
       
def main(args):
    
    rospy.init_node('simple_gui', anonymous=True)
    
    app = QtGui.QApplication(sys.argv)
    window = simple_gui()
    
    desktop = QtGui.QDesktopWidget()
    geometry = desktop.screenGeometry(1)
    window.move(geometry.left(), geometry.top())
    window.resize(geometry.width(), geometry.height())
    window.showFullScreen()
    
    sys.exit(app.exec_())
    
if __name__ == '__main__':
    try:
        main(sys.argv)
    except KeyboardInterrupt:
        print("Shutting down")
