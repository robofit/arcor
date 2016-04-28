#!/usr/bin/env python

import rospy
import rospkg
import sys
import signal

import cv2
from std_msgs.msg import String, Bool
from PyQt4 import QtGui, QtCore, QtOpenGL
from art_msgs.msg import InstancesArray,  UserStatus
from geometry_msgs.msg import PoseStamped, PointStamped
from std_srvs.srv import Empty, EmptyResponse
import tf

from helper_objects import scene_place,  scene_object,  pointing_point
import numpy as np
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image, CameraInfo
from image_geometry import PinholeCameraModel

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

       self.tfl = tf.TransformListener()
       
       self.inited = False
       
       rospack = rospkg.RosPack()
       self.img_path = rospack.get_path('art_simple_gui') + '/imgs'
       
       self.obj_sub = rospy.Subscriber('/art_object_detector/object_filtered', InstancesArray, self.object_cb)
       self.point_left_sub = rospy.Subscriber('/pointing_left', PoseStamped, self.pointing_point_left_cb)
       self.point_right_sub = rospy.Subscriber('/pointing_right', PoseStamped, self.pointing_point_right_cb)
       self.user_status_sub = rospy.Subscriber('/art_table_pointing/user_status',  UserStatus,  self.user_status_cb)
       
       self.selected_object_pub = rospy.Publisher("/art_simple_gui/selected_object", String, queue_size=10)
       self.selected_place_pub = rospy.Publisher("/art_simple_gui/selected_place", PoseStamped, queue_size=10)
       self.calibrated_pub = rospy.Publisher("/art_simple_gui/calibrated", Bool, queue_size=10, latch=True)
       
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

       self.h_matrix = None
       
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
       
       self.calibrated_pub.publish(Bool(False))

       self.inited = True

    def calibrate(self, req):

        self.emit(QtCore.SIGNAL('calibrate'))
        return EmptyResponse()

    def calibrate_evt(self):

        self.checkerboard.show()
        self.ctimer = QtCore.QTimer.singleShot(1000, self.calibrate_evt2)

    def calibrate_int(self):

        # TODO use message_filters to get synchronized messages!
        try:
          cam_info = rospy.wait_for_message('/kinect2/sd/camera_info', CameraInfo, 1.0)
        except rospy.ROSException:

            rospy.logerr("Could not get camera_info")
            return False

        try:
          img = rospy.wait_for_message('/kinect2/sd/image_color_rect', Image, 1.0)
        except rospy.ROSException:

            rospy.logerr("Could not get image")
            return False

        try:
          depth = rospy.wait_for_message('/kinect2/sd/image_depth_rect', Image, 1.0)
        except rospy.ROSException:

            rospy.logerr("Could not get depth image")
            return False

        rospy.loginfo("Got data...")

        try:
              cv_img = self.bridge.imgmsg_to_cv2(img, "bgr8")
        except CvBridgeError as e:
          print(e)
          return False

        try:
              cv_depth = self.bridge.imgmsg_to_cv2(depth)
        except CvBridgeError as e:
          print(e)
          return False

        cv_depth = cv2.medianBlur(cv_depth, 5)

        model = PinholeCameraModel()
        model.fromCameraInfo(cam_info)

        cv_img = cv2.cvtColor(cv_img, cv2.COLOR_BGR2GRAY)
        #cv2.imwrite('/home/zdenal/tmp/gui-calib/in.png', cv_img)

        ret, corners = cv2.findChessboardCorners(cv_img, (9,6), None, flags=cv2.CALIB_CB_ADAPTIVE_THRESH | cv2.CALIB_CB_FILTER_QUADS | cv2.CALIB_CB_NORMALIZE_IMAGE)

        if ret == False:

            rospy.logerr("Could not find chessboard corners")
            return False

        criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)
        cv2.cornerSubPix(cv_img,corners,(11,11),(-1,-1),criteria)
        corners = corners.reshape(1,-1,2)[0]

        points = []

        # take chessboard corners and make 3D points using projection and depth data
        for c in corners:

          pt = list(model.projectPixelTo3dRay((c[0], c[1])))

          # depth image is noise - let's make mean of few pixels
          da = []
          for x in range(int(c[0]) - 2, int(c[0]) + 3):
              for y in range(int(c[1]) - 2, int(c[1]) + 3):
                  da.append(cv_depth[y, x]/1000.0)

          d = np.mean(da)
          pt[:] = [x*d for x in pt]

          ps = PointStamped()
          ps.header.stamp = rospy.Time(0)
          ps.header.frame_id = img.header.frame_id
          ps.point.x = pt[0]
          ps.point.y = pt[1]
          ps.point.z = pt[2]

          # transform 3D point from camera into the world coordinates
          try:
              ps = self.tfl.transformPoint("marker", ps)
          except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
              rospy.logerr("can't get transform")
              return False

          # store x,y -> here we assume that points are 2D (on tabletop)
          # TODO skip point where z differs too much from zero?
          points.append([ps.point.x, ps.point.y])

        ppoints = []


        box_size = self.checkerboard.pixmap().width()/(9+2.0) # in pixels
        origin = (2*box_size, 2*box_size)

        # generate 2D (screen) points of chessboard corners (in pixels)
        for y in range(0,6):
          for x in range(0,9):
              px = (origin[0]-x*box_size)
              py = (origin[1]+y*box_size)
              ppoints.append([px, py])

        #print "points"
        #print points
        #print ""
        #print "ppoints"
        #print ppoints

        # find homography between points on table (in meters) and screen points (pixels)
        h, status = cv2.findHomography(np.array(points), np.array(ppoints), cv2.RANSAC, 5.0)

        self.h_matrix = np.matrix(h)

        rospy.loginfo("Calibrated!")

        #print self.h_matrix

        #cv2.drawChessboardCorners(cv_img, (9,6), corners, True)
        #cv2.imwrite('/home/zdenal/tmp/gui-calib/in.png', cv_img)
        #cv2.drawChessboardCorners(cv_depth, (9,6), corners, True)
        #cv2.imwrite('/home/zdenal/tmp/gui-calib/depth.png', cv_depth)

        return True

    def calibrate_evt2(self):

        cnt = 0
        ret = False
        while ret == False and cnt < 5:
            ret = self.calibrate_int()
            cnt += 1
        self.checkerboard.hide()

        self.calibrated_pub.publish(Bool(ret))
    
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

        if self.h_matrix is None:

            self.label.setPlainText('Waiting for calibration...')
            return
        
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

        if self.h_matrix is None:

            return

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
            self.viz_places.append(scene_place(self.scene,  pointed_place,  self.selected_place_pub, self.size(), self.h_matrix))
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

        if self.h_matrix is None:

            rospy.logerr("Not calibrated!")
            return None
    
        #px = int(self.width() - int((pose.position.x / self.marker_box_size) * self.height()/10.0))
        #py = int((pose.position.y / self.marker_box_size) * self.height()/10.0)
        #return (px, py)

        pt = p=np.array([[pose.position.x], [pose.position.y], [1]])

        px = self.h_matrix*pt

        px[0] = self.width() - px[0]

        return (px[0], px[1])
     
    def pointing_point_left_cb(self, msg):
        
        if self.h_matrix is None: return
        if not self.inited: return
       
        pos = self.get_px(msg.pose)
        self.emit(QtCore.SIGNAL('pointing_point_left'),  pos)
           
    def pointing_point_right_cb(self, msg):
        
        if self.h_matrix is None: return
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
