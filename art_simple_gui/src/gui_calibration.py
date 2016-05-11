import rospy
from PyQt4 import QtGui, QtCore
from geometry_msgs.msg import PoseStamped,  PoseArray,  PointStamped,  Pose
import numpy as np
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image, CameraInfo
from image_geometry import PinholeCameraModel
import cv2
import ast
from std_srvs.srv import Empty, EmptyResponse
from std_msgs.msg import Bool
import tf

class gui_calibration(QtCore.QObject):
    
    def __init__(self,  scene,  img_path,  size):
        
        super(gui_calibration, self).__init__()
        
        self.img_path = img_path
        self.scene = scene
        
        self.checkerboard_img = QtGui.QPixmap(self.img_path + "/pattern.png")
        self.checkerboard = self.scene.addPixmap(self.checkerboard_img.scaled(size, QtCore.Qt.KeepAspectRatio))
        self.checkerboard.setZValue(100)
        self.checkerboard.hide()
        
        self.bridge = CvBridge()
        
        self.calibrated_pub = rospy.Publisher("/art_simple_gui/calibrated", Bool, queue_size=10, latch=True)
        self.corners_pub = rospy.Publisher("/art_simple_gui/corners", PoseArray,  queue_size=10,  latch=True)
        self.srv_calibrate = rospy.Service('/art_simple_gui/calibrate', Empty, self.calibrate)
        
        self.size = size
        
        self.model = None
        self.h_matrix = None
        
        try:
            s = rospy.get_param("/art_simple_gui/calibration_matrix")
            self.h_matrix = np.matrix(ast.literal_eval(s))
            rospy.loginfo("Loaded calibration from param server")
        except KeyError:
            pass
            
        self.calibrated_pub.publish(Bool(self.h_matrix is not None))
        
        QtCore.QObject.connect(self, QtCore.SIGNAL('calibrate'), self.calibrate_evt)
        QtCore.QObject.connect(self, QtCore.SIGNAL('calibrate2'), self.calibrate_evt2)
        
    def resize(self,  size):
        
        self.size = size
        self.checkerboard.setPixmap(self.checkerboard_img.scaled(size, QtCore.Qt.KeepAspectRatio))
        
    def is_calibrated(self):
        
        return self.h_matrix is not None
    
    def get_px(self, pose):

        if self.h_matrix is None:

            rospy.logerr("Not calibrated!")
            return None

        pt = np.array([[pose.position.x], [pose.position.y], [1.0]])
        px = self.h_matrix*pt

        w = px[2].tolist()[0][0]
        x = px[0].tolist()[0][0]
        y = px[1].tolist()[0][0]
        
        return (self.size.width()-int(round(x/w)), int(round(y/w)))
    
    def get_pose(self,  px,  py):
        
        ps = PoseStamped()
        ps.header.frame_id = "marker"
        ps.header.stamp = rospy.Time.now()
        
        p = np.array([[self.size.width()-px], [py], [1.0]])
        res = np.linalg.inv(self.h_matrix)*p

        ps.pose.position.x = float(res[0]/res[2])
        ps.pose.position.y = float(res[1]/res[2])
        ps.pose.position.z = 0.0

        ps.pose.orientation.x = 0.0
        ps.pose.orientation.y = 0.0
        ps.pose.orientation.z = 0.0
        ps.pose.orientation.w = 1.0
        
        return ps
        
    def get_caminfo(self):
        
        cam_info = None
        try:
          cam_info = rospy.wait_for_message('camera_info', CameraInfo, 1.0)
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
              img = rospy.wait_for_message('camera_image', Image, 1.0)
            except rospy.ROSException:

                rospy.logerr("Could not get image")
                return False

            try:
              depth = rospy.wait_for_message('camera_depth_image', Image, 1.0)
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
                    px = self.size.width()-(origin[0]+x*box_size)
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
