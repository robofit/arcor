#!/usr/bin/env python

import rospy
import rospkg
import sys
import signal

import cv2
from std_msgs.msg import String
from PyQt4 import QtGui, QtCore, QtOpenGL
from art_msgs.msg import InstancesArray,  UserStatus
from geometry_msgs.msg import PoseStamped
from std_srvs.srv import Empty, EmptyResponse

from helper_objects import scene_place,  scene_object,  pointing_point
import numpy as np
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image

# TODO modes of operation (states) - currently only pick (object selection) and place (place selection)
# TODO step back btn / some menu?
# TODO notifications / state information (smart label)
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
       self.user_status_sub = rospy.Subscriber('/art_table_pointing/user_status',  UserStatus,  self.user_status_cb)
       
       self.selected_object_pub = rospy.Publisher("/art_simple_gui/selected_object", String, queue_size=10)
       self.selected_place_pub = rospy.Publisher("/art_simple_gui/selected_place", PoseStamped, queue_size=10)
       
       self.srv_calibrate = rospy.Service('/art_simple_gui/calibrate', Empty, self.calibrate)
       self.srv_show_marker = rospy.Service('/art_simple_gui/show_marker', Empty, self.show_marker)
       self.srv_hide_marker = rospy.Service('/art_simple_gui/hide_marker', Empty, self.hide_marker)
       self.srv_clear_all = rospy.Service('/art_simple_gui/clear_all', Empty, self.clear_all) # clear all selections etc.
       
       self.marker_box_size = rospy.get_param("/art_params/marker_box_size", 0.0685)
       
       self.objects = None
       self.viz_objects = {}
       self.viz_places = []
       
       self.object_selected = False
       self.object_selected_at = None
       self.place_selected = False
       
       self.scene=QtGui.QGraphicsScene(self)
       self.scene.setBackgroundBrush(QtCore.Qt.black)
       self.view = QtGui.QGraphicsView(self.scene, self)
       self.view.setRenderHint(QtGui.QPainter.Antialiasing)
       self.view.setViewportUpdateMode(QtGui.QGraphicsView.FullViewportUpdate)
       #self.view.setViewport(QtOpenGL.QGLWidget()) # rendering using OpenGL -> somehow broken :(
       
       self.pm = QtGui.QPixmap(self.img_path + "/koberec.png")
       self.marker = self.scene.addPixmap(self.pm.scaled(self.size(), QtCore.Qt.KeepAspectRatio))
       self.marker.setZValue(-100)
       self.marker.hide()

       self.checkerboard_img = QtGui.QPixmap(self.img_path + "/pattern.png")
       self.checkerboard = self.scene.addPixmap(self.checkerboard_img.scaled(self.size(), QtCore.Qt.KeepAspectRatio))
       self.checkerboard.setZValue(100)
       self.checkerboard.hide()
       self.bridge = CvBridge()
       
       self.resizeEvent = self.on_resize
       
       self.pointing_left = pointing_point("left", self.scene)
       self.pointing_right = pointing_point("right", self.scene)
       
       QtCore.QObject.connect(self, QtCore.SIGNAL('objects'), self.objects_evt)
       QtCore.QObject.connect(self, QtCore.SIGNAL('pointing_point_left'), self.pointing_point_left_evt)
       QtCore.QObject.connect(self, QtCore.SIGNAL('pointing_point_right'), self.pointing_point_right_evt)
       QtCore.QObject.connect(self, QtCore.SIGNAL('clear_all()'), self.clear_all_evt)
       QtCore.QObject.connect(self, QtCore.SIGNAL('show_marker()'), self.show_marker_evt)
       QtCore.QObject.connect(self, QtCore.SIGNAL('hide_marker()'), self.hide_marker_evt)
       QtCore.QObject.connect(self, QtCore.SIGNAL('user_status'), self.user_status_evt)
       QtCore.QObject.connect(self, QtCore.SIGNAL('calibrate'), self.calibrate_evt)
       QtCore.QObject.connect(self, QtCore.SIGNAL('calibrate2'), self.calibrate_evt2)
       
       self.timer = QtCore.QTimer()
       self.timer.start(500)
       self.timer.timeout.connect(self.timer_evt)
       
       # TODO "smart" label - able to show messages for defined time, more messages at the time, images (?) etc.
       self.label = self.scene.addText("Waiting for user",  QtGui.QFont('Arial', 26))
       self.label.rotate(180)
       self.label.setDefaultTextColor(QtCore.Qt.white)
       self.label.setZValue(200)
       
       self.user_status = None
       
       self.ignored_items = [self.label,  self.marker, self.checkerboard]
       
       self.inited = True

    def calibrate(self, req):

        self.emit(QtCore.SIGNAL('calibrate'))
        return EmptyResponse()

    def calibrate_evt(self):

        self.checkerboard.show()
        self.ctimer = QtCore.QTimer.singleShot(10000, self.calibrate_evt2)

    def calibrate_evt2(self):

      try:
        img = rospy.wait_for_message('/kinect2/sd/image_color_rect', Image, 2)
      except rospy.ROSException:

          rospy.logerr("Could not get image")
          self.checkerboard.hide()
          return

      try:
            cv_img = self.bridge.imgmsg_to_cv2(img, "bgr8")
      except CvBridgeError as e:
        print(e)
        self.checkerboard.hide()
        return

      cv_img = cv2.cvtColor(cv_img,cv2.COLOR_BGR2GRAY)
      ret, corners = cv2.findChessboardCorners(cv_img, (9,6),None)

      print cv_img.shape

      if ret == False:

          rospy.logerr("Could not find chessboard corners")
          self.checkerboard.hide()
	  return

      #cv2.drawChessboardCorners(cv_img, (9,6), corners, True)
      #cv2.imshow('image',cv_img)

      #criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)
      #corners2 = cv2.cornerSubPix(cv_img,corners,(11,11),(-1,-1),criteria)
      #print corners2

      print "self.checkerboard.pixmap().width: " + str(self.checkerboard.pixmap().width())
      print "self.width(): " + str(self.width())

      box_size = self.checkerboard.pixmap().width()/12

      print "box_size: " + str(box_size)

      pcorners = []

      for x in range(2*box_size, self.checkerboard.pixmap().width()-2*box_size, box_size):
          for y in range(2*box_size, self.checkerboard.pixmap().height()-2*box_size, box_size):
            pcorners.append([x,y])

      corners = corners.reshape(1,-1,2)[0]
      pcorners = np.array(pcorners, dtype=corners.dtype)

      #print corners
      #print corners.shape
      #print("")
      #print pcorners
      #print pcorners.shape

      # warpPerspective(im_src, im_out, h.inv(), im_dst.size()); => transform source to destination
      # warpPerspective(im_dst, im_out, h, im_dst.size()); => transform destination to source

      h, status = cv2.findHomography(corners, pcorners, cv2.RANSAC, 5.0)

      print h
      #print status

      img_src = cv2.imread(self.img_path + "/pattern.png", 0)
      img_src = cv2.resize(img_src, None, fx=0.5, fy=0.5, interpolation = cv2.INTER_CUBIC)

      img_dst = cv2.warpPerspective(img_src, h, (img_src.shape[1], img_src.shape[0]))

      print img_src.shape
      print img_dst.shape
      #cv2.imshow('image', img_dst)
      #cv2.waitKey(0)
      #cv2.destroyAllWindows()

      self.checkerboard.hide()


    
    def user_status_cb(self,  msg):
        
        self.emit(QtCore.SIGNAL('user_status'),  msg)
    
    def user_status_evt(self,  msg):
    
        self.user_status = msg
        
        if self.user_status.header.stamp == rospy.Time(0):
            self.user_status.header.stamp = rospy.Time.now()

    def show_marker(self, req):
        
        self.emit(QtCore.SIGNAL('show_marker()'))
        return EmptyResponse()
        
    def show_marker_evt(self):
        
        self.marker.show()
        
    def hide_marker(self, req):
        
        self.emit(QtCore.SIGNAL('hide_marker()'))
        return EmptyResponse()
    
    def hide_marker_evt(self):
        
        self.marker.hide()
    
    def clear_all_evt(self):
        
        self.object_selected = False
        self.place_selected = False
        self.label.setPlainText("Waiting for user")
    
        for k,  v in self.viz_objects.iteritems():
            
            v.unselect()
    
        for it in self.viz_places:
            it.remove()
        self.viz_places = []
        
    def clear_all(self, req):
    
        self.emit(QtCore.SIGNAL('clear_all()'))
        return EmptyResponse()
       
    def timer_evt(self):
        
        if self.user_status is not None and rospy.Time.now() - self.user_status.header.stamp > rospy.Duration(2):
            
            self.user_status = None
        
        if self.user_status is None:
            
            self.label.setPlainText('Waiting for user tracking...')
    
        elif self.user_status.user_state == UserStatus.NO_USER:
            
            self.label.setPlainText('Waiting for user...')
            
        elif self.user_status.user_state == UserStatus.USER_NOT_CALIBRATED:
            
            self.label.setPlainText('Please make a calibration pose')
            
        elif (self.user_status.user_state == UserStatus.USER_CALIBRATED) and not (self.pointing_left.is_active() or self.pointing_right.is_active()):
            
            self.label.setPlainText('Point at objects or places to select them')
    
    def on_resize(self, event):
    
        self.view.setFixedSize(self.width(), self.height())
        self.view.setSceneRect(QtCore.QRectF(0, 0, self.width(), self.height()))
        self.view.setHorizontalScrollBarPolicy(QtCore.Qt.ScrollBarAlwaysOff);
        self.view.setVerticalScrollBarPolicy(QtCore.Qt.ScrollBarAlwaysOff);
        self.marker.setPixmap(self.pm.scaled(self.size(), QtCore.Qt.KeepAspectRatio))
        self.checkerboard.setPixmap(self.checkerboard_img.scaled(self.size(), QtCore.Qt.KeepAspectRatio))
        self.label.setPos(self.width() - 70,  70)
      
    def object_cb(self, msg):

        self.emit(QtCore.SIGNAL('objects'),  msg)
    
    def pointing_point_left_evt(self,  pos):

        if (pos[0] in range(0, self.width()) and pos[1] in range(0, self.height())):
                
           self.pointing_left.set_pos(pos)
           self.pointing_point(self.pointing_left)
           
    def pointing_point_right_evt(self,  pos):
        
        if (pos[0] in range(0, self.width()) and pos[1] in range(0, self.height())):
           
           self.pointing_right.set_pos(pos)
           self.pointing_point(self.pointing_right)
    
    def pointing_point(self,  pt):
        
        if self.place_selected:

            self.label.setPlainText("Wait please...")
            return
        
        if self.user_status is None or self.user_status.user_state != UserStatus.USER_CALIBRATED:
            return
        
        if not self.object_selected:
            
            self.label.setPlainText("Select an object")
             
        else:
            
            self.label.setPlainText("Select a place")
            if rospy.Time.now() - self.object_selected_at < rospy.Duration(2): return
        
        if not pt.is_active() or pt.viz is None: return
        
        if not self.object_selected:
            
            for k, v in self.viz_objects.iteritems():
                
                if v.pointing(pt.viz) is True:
                    
                    rospy.loginfo("Object selected, now select place") # TODO "attach" object shape to the pointing point(s)?
                    self.object_selected = True
                    self.object_selected_at = rospy.Time.now()
                    break
        
        if self.object_selected is False: return
        
        items = self.scene.collidingItems(pt.viz)
        
        for iit in self.ignored_items: # TODO test it
            
            if iit in items: items.remove(iit)
        
        pointed_place = pt.get_pointed_place()
        
        if len(items) == 0 and (pointed_place is not None):
            
            # TODO how to keep some minimal spacing between places?
            skip = False
            for pl in self.viz_places:
                
                if np.linalg.norm(np.array(pl.pos) - np.array(pointed_place)) < 150:
                    
                    rospy.logwarn(pt.id + ": place near to x=" + str(pointed_place[0]) + ", y=" + str(pointed_place[1]) + " already exists")
                    skip = True
                    break

            if skip: return

            rospy.loginfo(pt.id + ": new place selected at x=" + str(pointed_place[0]) + ", y=" + str(pointed_place[1]))
            self.viz_places.append(scene_place(self.scene,  pointed_place,  self.marker_box_size,  self.selected_place_pub, self.size()))
            self.place_selected = True
        
    def objects_evt(self,  msg):
    
       self.objects = msg.instances
    
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
    
       px = int(self.width() - int((pose.position.x / self.marker_box_size) * self.height()/10.0))
       py = int((pose.position.y / self.marker_box_size) * self.height()/10.0)
       
       return (px, py)
     
    def pointing_point_left_cb(self, msg):
        
        if not self.inited: return
       
        pos = self.get_px(msg.pose)
        self.emit(QtCore.SIGNAL('pointing_point_left'),  pos)
           
    def pointing_point_right_cb(self, msg):
        
        if not self.inited: return
       
        pos = self.get_px(msg.pose)
        self.emit(QtCore.SIGNAL('pointing_point_right'),  pos)
       
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
