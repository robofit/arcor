#!/usr/bin/env python

import rospy
import rospkg
import sys
import signal
import ast

import cv2
from std_msgs.msg import String, Bool
from PyQt4 import QtGui, QtCore, QtOpenGL
from art_msgs.msg import InstancesArray,  UserStatus
from art_msgs.srv import getProgram
from art_msgs.msg import RobotProgramAction, RobotProgramFeedback,  RobotProgramGoal
from geometry_msgs.msg import Pose,  PoseStamped, PointStamped,  PoseArray
from std_srvs.srv import Empty, EmptyResponse
import tf
import actionlib

from helper_objects import scene_place,  scene_object,  pointing_point
from program_widget import program_widget
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

        self.tfl = None

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
        
        self.corners_pub = rospy.Publisher("/art_simple_gui/corners", PoseArray,  queue_size=10,  latch=True)

        self.srv_calibrate = rospy.Service('/art_simple_gui/calibrate', Empty, self.calibrate)
        self.srv_show_marker = rospy.Service('/art_simple_gui/show_marker', Empty, self.show_marker)
        self.srv_hide_marker = rospy.Service('/art_simple_gui/hide_marker', Empty, self.hide_marker)
        self.srv_clear_all = rospy.Service('/art_simple_gui/clear_all', Empty, self.clear_all) # clear all selections etc.

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

        self.pm = QtGui.QPixmap(self.img_path + "/koberec.png") # TODO use homography matrix to correct it
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

        self.model = None
        self.h_matrix = None
        
        try:
            s = rospy.get_param("/art_simple_gui/calibration_matrix")
            self.h_matrix = np.matrix(ast.literal_eval(s))
            rospy.loginfo("Loaded calibration from param server")
        except KeyError:
            pass
            
        self.calibrated_pub.publish(Bool(self.h_matrix is not None))

        self.prog = program_widget(self)
        self.prog.resize(400, 200)
        self.prog.move(10, 10)
        self.prog.show()
    
        self.brain_client = actionlib.SimpleActionClient("/art_brain/do_program", RobotProgramAction)
        
        # TODO only for testing - program should be selected by user
        self.load_program(0)
        self.start_program()
        
        self.inited = True

    def program_feedback(self,  msg):
        
        self.prog.set_current(msg.current_program,  msg.current_item)

    def program_done(self,  status,  msg):
        
        # TODO do something meaningful
         self.start_program()
        
    def start_program(self):
        
        self.brain_client.wait_for_server()
        goal = RobotProgramGoal()
        goal.program_array.programs.append(self.prog.prog)
        self.brain_client.send_goal(goal,  done_cb=self.program_done,  feedback_cb=self.program_feedback)
        
    def load_program(self,  prog_id,  template = False):
        
        rospy.wait_for_service('/art_db/program/get')
    
        try:
            prog_srv = rospy.ServiceProxy('/art_db/program/get', getProgram)
            resp = prog_srv(0)
            self.prog.set_prog(resp.program)
        except rospy.ServiceException, e:
            print "Service call failed: %s"%e

    def get_caminfo(self):
        
        cam_info = None
        try:
          cam_info = rospy.wait_for_message('/kinect2/hd/camera_info', CameraInfo, 1.0)
        except rospy.ROSException:

          rospy.logerr("Could not get camera_info")
        
        if cam_info is not None:
            self.model = PinholeCameraModel()
            self.model.fromCameraInfo(cam_info)

    def calibrate(self, req):

        self.tfl = tf.TransformListener()
        # TODO subscribe to image/depth (message_filters?) and call calibrate_evt2 from there
        self.emit(QtCore.SIGNAL('calibrate'))
        return EmptyResponse()

    def calibrate_evt(self):

        self.checkerboard.show()
        self.ctimer = QtCore.QTimer.singleShot(1000, self.calibrate_evt2)

    def calibrate_int(self):

        points = []
        ppoints = []
        
        cnt = 0
        
        box_size = self.checkerboard.pixmap().width()/(10+2.0) # in pixels
        origin = (2*box_size, 2*box_size) # origin of the first corner
        
        ppp = PoseArray()
        ppp.header.stamp = rospy.Time.now()
        ppp.header.frame_id = "marker"
        
        while(cnt < 3):
            
            cnt += 1
        
            try:
              img = rospy.wait_for_message('/kinect2/hd/image_color_rect', Image, 1.0)
            except rospy.ROSException:

                rospy.logerr("Could not get image")
                return False

            try:
              depth = rospy.wait_for_message('/kinect2/hd/image_depth_rect', Image, 1.0)
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
            cv_img = cv2.cvtColor(cv_img, cv2.COLOR_BGR2GRAY)

            ret, corners = cv2.findChessboardCorners(cv_img, (9,6), None, flags=cv2.CALIB_CB_ADAPTIVE_THRESH | cv2.CALIB_CB_FILTER_QUADS | cv2.CALIB_CB_NORMALIZE_IMAGE)

            if ret == False:

                rospy.logerr("Could not find chessboard corners")
                return False

            criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 100, 0.001)
            cv2.cornerSubPix(cv_img,corners,(11,11),(-1,-1),criteria)
            corners = corners.reshape(1,-1,2)[0]

            # take chessboard corners and make 3D points using projection and depth data
            for c in corners:

              pt = list(self.model.projectPixelTo3dRay((c[0], c[1])))
              pt[:] = [x/pt[2] for x in pt]

              # depth image is noisy - let's make mean of few pixels
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
              
              pp = Pose()
              pp.position.x = ps.point.x
              pp.position.y = ps.point.y
              pp.position.z = ps.point.z
              pp.orientation.x = 0
              pp.orientation.y = 0
              pp.orientation.z = 0
              pp.orientation.w = 1.0
              
              ppp.poses.append(pp)

              # store x,y -> here we assume that points are 2D (on tabletop)
              points.append([ps.point.x, ps.point.y])

            # generate 2D (screen) points of chessboard corners (in pixels)
            for y in range(0,6):
              for x in range(0,9):
                    px = self.width()-(origin[0]+x*box_size)
                    py = (origin[1]+y*box_size)
                    ppoints.append([px, py])
        
        self.corners_pub.publish(ppp)
        
        # find homography between points on table (in meters) and screen points (pixels)
        h, status = cv2.findHomography(np.array(points), np.array(ppoints), cv2.LMEDS)
        
        self.h_matrix = np.matrix(h)
        self.box_size = box_size
        self.pm_width =  self.checkerboard.pixmap().width()

        rospy.loginfo("Calibrated!")

        # store homography matrix to parameter server
        s = str(self.h_matrix.tolist())
        rospy.set_param("/art_simple_gui/calibration_matrix",  s)
        print s
        
        return True

    def calibrate_evt2(self):

        cnt = 0

        if self.model is None:
            self.get_caminfo()
            if self.model is None:
                rospy.logerr("No camera_info -> cannot calibrate")
                cnt = 10
        
        ret = False
        while ret == False and cnt < 5:
            ret = self.calibrate_int()
            cnt += 1
        self.checkerboard.hide()
        self.tfl = None
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

        if not self.inited: return
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

        pt = np.array([[pose.position.x], [pose.position.y], [1.0]])
        px = self.h_matrix*pt

        w = px[2].tolist()[0][0]
        x = px[0].tolist()[0][0]
        y = px[1].tolist()[0][0]
        
        return (self.width()-int(round(x/w)), int(round(y/w)))
     
    def pointing_point_left_cb(self, msg):
        
        if not self.inited: return
        if self.h_matrix is None: return
       
        pos = self.get_px(msg.pose)
        self.emit(QtCore.SIGNAL('pointing_point_left'),  pos)
           
    def pointing_point_right_cb(self, msg):
        
        if not self.inited: return
        if self.h_matrix is None: return
       
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
