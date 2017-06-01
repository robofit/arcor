#!/usr/bin/env python

from PyQt4 import QtGui, QtCore, QtNetwork
import rospkg
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image, CameraInfo
import rospy
import cv2
import numpy as np
import qimage2ndarray
from std_msgs.msg import Bool
from std_srvs.srv import Trigger, TriggerResponse
import message_filters
from image_geometry import PinholeCameraModel
from geometry_msgs.msg import PointStamped, Pose, PoseArray
import tf
import ast
# import time

# TODO create ProjectorROS (to separate QT / ROS stuff)
# podle vysky v pointcloudu / pozice projektoru se vymaskuji mista kde je
# neco vyssiho - aby se promitalo jen na plochu stolu ????


class Projector(QtGui.QWidget):

    def __init__(self):

        super(Projector, self).__init__()

        self.proj_id = rospy.get_param('~projector_id', 'test')
        self.world_frame = rospy.get_param('~world_frame', 'marker')
        self.screen = rospy.get_param('~screen_number', 0)
        self.camera_image_topic = rospy.get_param(
            '~camera_image_topic', 'kinect2/hd/image_color_rect')
        self.camera_depth_topic = rospy.get_param(
            '~camera_depth_topic', 'kinect2/hd/image_depth_rect')
        self.camera_info_topic = rospy.get_param(
            '~camera_info_topic', 'kinect2/hd/camera_info')

        self.map_x = None
        self.map_y = None

        self.rpm = rospy.get_param('rpm')
        self.scene_size = rospy.get_param("scene_size")
        self.scene_origin = rospy.get_param("scene_origin")

        rospy.loginfo("Projector '" + self.proj_id +
                      "', on screen " + str(self.screen))

        img_path = rospkg.RosPack().get_path('art_projected_gui') + '/imgs'
        self.checkerboard_img = QtGui.QPixmap(img_path + "/pattern.png")
        self.calibrating = False
        self.calibrated = False
        self.maps_ready = False

        desktop = QtGui.QDesktopWidget()
        geometry = desktop.screenGeometry(self.screen)
        self.move(geometry.left(), geometry.top())
        self.resize(geometry.width(), geometry.height())

        self.tfl = None
        self.bridge = CvBridge()

        self.setAutoFillBackground(True)
        p = self.palette()
        p.setColor(self.backgroundRole(), QtCore.Qt.black)
        self.setPalette(p)

        self.pix_label = QtGui.QLabel(self)
        self.pix_label.setAlignment(
            QtCore.Qt.AlignHCenter | QtCore.Qt.AlignVCenter)
        # self.pix_label.setScaledContents(True)
        self.pix_label.resize(self.size())

        self.server = rospy.get_param("scene_server")
        self.port = rospy.get_param("scene_server_port")
        self.tcpSocket = QtNetwork.QTcpSocket(self)
        self.blockSize = 0
        self.tcpSocket.readyRead.connect(self.getScene)
        self.tcpSocket.error.connect(self.on_error)

        self.projectors_calibrated_sub = rospy.Subscriber(
            '/art/interface/projected_gui/app/projectors_calibrated', Bool, self.projectors_calibrated_cb, queue_size=10)
        self.projectors_calibrated = False

        self.srv_calibrate = rospy.Service(
            "~calibrate", Trigger, self.calibrate_srv_cb)
        self.corners_pub = rospy.Publisher(
            "~corners", PoseArray, queue_size=10, latch=True)

        QtCore.QObject.connect(self, QtCore.SIGNAL(
            'show_chessboard'), self.show_chessboard_evt)
        QtCore.QObject.connect(self, QtCore.SIGNAL(
            'show_pix_label'), self.show_pix_label_evt)

        self.setWindowFlags(QtCore.Qt.WindowStaysOnTopHint)
        self.showFullScreen()
        self.setCursor(QtCore.Qt.BlankCursor)

        self.calibrated_pub = rospy.Publisher(
            "~calibrated", Bool, queue_size=1, latch=True)

        h_matrix = rospy.get_param("~calibration_matrix", None)

        if h_matrix is not None:
            rospy.loginfo('Loaded calibration from param.')
            self.calibrated = True
            self.calibrated_pub.publish(self.is_calibrated())
            self.init_map_from_matrix(np.matrix(ast.literal_eval(h_matrix)))
        else:
            self.calibrated_pub.publish(self.is_calibrated())
            
        self.connect()

    def init_map_from_matrix(self, m):

        rospy.loginfo("Building map from calibration matrix...")
        
        self.maps_ready = False

        Hd = self.height()
        Wd = self.width()

        self.map_x = np.zeros((Hd, Wd), np.float32)
        self.map_y = np.zeros((Hd, Wd), np.float32)

        m = np.linalg.inv(m)

        for y in range(0, int(Hd - 1)):
            for x in range(0, int(Wd - 1)):

                self.map_x.itemset(
                    (y, x), (m[0, 0] * x + m[0, 1] * y + m[0, 2]) / (m[2, 0] * x + m[2, 1] * y + m[2, 2]))
                self.map_y.itemset(
                    (y, x), (m[1, 0] * x + m[1, 1] * y + m[1, 2]) / (m[2, 0] * x + m[2, 1] * y + m[2, 2]))

        self.map_x, self.map_y = cv2.convertMaps(
            self.map_x, self.map_y, cv2.CV_16SC2)
        self.maps_ready = True
        rospy.loginfo("Done!")

    def show_pix_label_evt(self, show):

        if show:
            self.pix_label.clear()
            self.pix_label.show()
        else:
            self.pix_label.hide()

    def projectors_calibrated_cb(self, msg):

        self.projectors_calibrated = msg.data
        self.emit(QtCore.SIGNAL('show_pix_label'), self.projectors_calibrated)

    def connect(self):

        r = rospy.Rate(1.0 / 5)

        while not self.tcpSocket.waitForConnected(1):

            if rospy.is_shutdown():
                return
            rospy.loginfo("Waiting for scene server...")
            self.tcpSocket.connectToHost(self.server, self.port)
            r.sleep()

        rospy.loginfo('Connected to scene server.')

    def on_error(self):

        rospy.logerr("socket error")
        QtCore.QTimer.singleShot(0, self.connect)

    def getScene(self):

        instr = QtCore.QDataStream(self.tcpSocket)
        instr.setVersion(QtCore.QDataStream.Qt_4_0)

        while True:

            if self.blockSize == 0:
                if self.tcpSocket.bytesAvailable() < 4:
                    return

                self.blockSize = instr.readUInt32()

            if self.tcpSocket.bytesAvailable() < self.blockSize:
                return

            self.blockSize = 0

            pix = QtGui.QImage()
            ba = QtCore.QByteArray()
            instr >> ba

            # skip this frame if there is another one in buffer
            if self.tcpSocket.bytesAvailable() > 0:
                rospy.logdebug("Frame dropped")
                continue

            # start = time.time()

            # 16ms
            if not pix.loadFromData(ba, "JPG"):

                rospy.logerr("Failed to load image from received data")
                return

            if self.calibrating or not self.projectors_calibrated or not self.maps_ready:
                return

            # 3ms
            img = pix.convertToFormat(QtGui.QImage.Format_ARGB32)
            v = qimage2ndarray.rgb_view(img)

            # TODO some further optimalization? this is about 30ms (with INTER_LINEAR)s...
            image_np = cv2.remap(v, self.map_x, self.map_y, cv2.INTER_LINEAR)

            # this is about 3ms
            height, width, channel = image_np.shape
            bytesPerLine = 3 * width
            image = QtGui.QPixmap.fromImage(QtGui.QImage(
                image_np.data, width, height, bytesPerLine, QtGui.QImage.Format_RGB888))

            self.pix_label.setPixmap(image)
            self.update()

            # end = time.time()
            # rospy.logdebug("Frame loaded in: " + str(end-start))

            return

    def calibrate(self, image, info, depth):

        model = PinholeCameraModel()
        model.fromCameraInfo(info)

        try:
            cv_img = self.bridge.imgmsg_to_cv2(image, "bgr8")
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

        ret, corners = cv2.findChessboardCorners(
            cv_img, (9, 6), None, flags=cv2.CALIB_CB_ADAPTIVE_THRESH | cv2.CALIB_CB_FILTER_QUADS | cv2.CALIB_CB_NORMALIZE_IMAGE)

        if not ret:

            rospy.logerr("Could not find chessboard corners")
            return False

        criteria = (cv2.TERM_CRITERIA_EPS +
                    cv2.TERM_CRITERIA_MAX_ITER, 100, 0.001)
        cv2.cornerSubPix(cv_img, corners, (11, 11), (-1, -1), criteria)
        corners = corners.reshape(1, -1, 2)[0]

        points = []
        ppoints = []

        ppp = PoseArray()
        ppp.header.stamp = rospy.Time.now()
        ppp.header.frame_id = self.world_frame

        for c in corners:

            pt = list(model.projectPixelTo3dRay((c[0], c[1])))
            pt[:] = [x / pt[2] for x in pt]

            # depth image is noisy - let's make mean of few pixels
            da = []
            for x in range(int(c[0]) - 2, int(c[0]) + 3):
                for y in range(int(c[1]) - 2, int(c[1]) + 3):
                    da.append(cv_depth[y, x] / 1000.0)

            d = np.mean(da)
            pt[:] = [x * d for x in pt]

            ps = PointStamped()
            ps.header.stamp = rospy.Time(0)
            ps.header.frame_id = image.header.frame_id
            ps.point.x = pt[0]
            ps.point.y = pt[1]
            ps.point.z = pt[2]

            # transform 3D point from camera into the world coordinates
            try:
                ps = self.tfl.transformPoint(self.world_frame, ps)
            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                rospy.logerr("Can't get transform")
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
            points.append([self.rpm * ps.point.x, self.rpm * ps.point.y])

        self.corners_pub.publish(ppp)

        dx = (self.pix_label.width() - self.pix_label.pixmap().width()) / 2.0
        dy = (self.pix_label.height() - self.pix_label.pixmap().height()) / 2.0
        box_size = self.pix_label.pixmap().width() / 12.0

        # TODO self.scene_origin ???
        # generate requested table coordinates
        for y in range(0, 6):
            for x in range(0, 9):

                px = 2 * box_size + x * box_size + dx
                py = 2 * box_size + y * box_size + dy

                ppoints.append([px, py])

        h, status = cv2.findHomography(
            np.array(points), np.array(ppoints), cv2.LMEDS)

        h_matrix = np.matrix(h)

        self.emit(QtCore.SIGNAL('show_pix_label'), False)  # hide chessboard
        self.calibrating = False
        self.calibrated = True
        self.calibrated_pub.publish(self.is_calibrated())   

        # store homography matrix to parameter server
        s = str(h_matrix.tolist())
        rospy.set_param("~calibration_matrix", s)

        self.init_map_from_matrix(h_matrix)
        # self.h_matrix = np.matrix([[1,  0,  0], [0,  1,  0], [0,  0, 1.0]])

        return True

    def shutdown_ts(self):

        self.timeout_timer.shutdown()

        # TODO is this correct way how to shut it down?
        for sub in self.subs:
            sub.sub.unregister()
        self.ts = None

    def sync_cb(self, image, cam_info, depth):

        self.timeout_timer.shutdown()
        
        try:
            self.tfl.waitForTransform(self.world_frame, image.header.frame_id, rospy.Time(0), rospy.Duration(0.1))
        except tf.Exception:
            rospy.logwarn("Waiting for transform...")
            return

        self.emit(QtCore.SIGNAL('show_chessboard'))

        if self.calibrate(image, cam_info, depth):

            rospy.loginfo('Calibrated')

        else:

            self.calibration_attempts += 1
            
            rospy.logerr('Calibration failed, attempt: ' + str(self.calibration_attempts))

            if self.calibration_attempts < 10:
                self.timeout_timer = rospy.Timer(rospy.Duration(
                    3.0), self.timeout_timer_cb, oneshot=True)
                return

            rospy.logerr('Giving up.')
            self.calibrating = False
            self.emit(QtCore.SIGNAL('show_pix_label'), False)
            self.calibrated_pub.publish(self.is_calibrated())

        self.shutdown_ts()
        if self.is_calibrated():
            self.tfl = None

    def show_chessboard_evt(self):
        
        if self.pix_label.isVisible():
            return

        rat = 1.0  # TODO make checkerboard smaller and smaller if it cannot be detected
        self.pix_label.show()
        self.pix_label.setPixmap(self.checkerboard_img.scaled(
            rat * self.width(), rat * self.height(), QtCore.Qt.KeepAspectRatio))

    def timeout_timer_cb(self, evt):

        rospy.logerr("Timeout - no message arrived or transformation is not available.")
        self.shutdown_ts()
        self.calibrated_pub.publish(self.is_calibrated())
        self.calibrating = False

    def tfl_delay_timer_cb(self, evt=None):

        rospy.loginfo('Subscribing to camera topics: ' + str(
            [self.camera_image_topic, self.camera_info_topic, self.camera_depth_topic]))

        self.subs = []
        self.subs.append(message_filters.Subscriber(
            self.camera_image_topic, Image))
        self.subs.append(message_filters.Subscriber(
            self.camera_info_topic, CameraInfo))
        self.subs.append(message_filters.Subscriber(
            self.camera_depth_topic, Image))

        self.ts = message_filters.TimeSynchronizer(self.subs, 10)
        self.ts.registerCallback(self.sync_cb)

        self.timeout_timer = rospy.Timer(rospy.Duration(
            3.0), self.timeout_timer_cb, oneshot=True)

    def calibrate_srv_cb(self, req):

        resp = TriggerResponse()
        resp.success = True

        if self.calibrating:
            msg = 'Calibration already running'
            rospy.logwarn(msg)
            resp.message = msg
            resp.success = False
            return resp

        rospy.loginfo('Starting calibration')
        self.calibrating = True

        self.calibration_attempts = 0

        # TF Listener needs some time to buffer data
        if self.tfl is None:
            self.tfl = tf.TransformListener()
            self.tfl_timer = rospy.Timer(rospy.Duration(
                3.0), self.tfl_delay_timer_cb, oneshot=True)
        else:
            self.tfl_delay_timer_cb()

        return resp

    def is_calibrated(self):

        return self.calibrated

    def on_resize(self, event):

        self.pix_label.resize(self.size())
