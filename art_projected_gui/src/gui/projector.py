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
from std_srvs.srv import Empty, EmptyResponse
import message_filters
from image_geometry import PinholeCameraModel
from geometry_msgs.msg import PointStamped, Pose, PoseArray
import tf

# TODO create ProjectorROS (to separate QT / ROS stuff)
# podle vysky v pointcloudu / pozice projektoru se vymaskuji mista kde je neco vyssiho - aby se promitalo jen na plochu stolu ????


class Projector(QtGui.QWidget):

    def __init__(self):

        super(Projector, self).__init__()

        self.proj_id = rospy.get_param('~projector_id', 'test')
        self.world_frame = rospy.get_param('~world_frame', 'marker')
        self.screen = rospy.get_param('~screen_number', 0)
        self.camera_image_topic = rospy.get_param('~camera_image_topic', '/kinect2/qhd/image_color_rect')
        self.camera_depth_topic = rospy.get_param('~camera_depth_topic', '/kinect2/qhd/image_depth_rect')
        self.camera_info_topic = rospy.get_param('~camera_info_topic', '/kinect2/qhd/camera_info')

        self.h_matrix = rospy.get_param('~calibration_matrix', None)
        self.rpm = rospy.get_param('~rpm', 1280)
        self.scene_size = rospy.get_param("~scene_size", [1.2, 0.64])
        self.scene_origin = rospy.get_param("~scene_origin", [0, 0])

        rospy.loginfo("Projector '" + self.proj_id + "', on screen " + str(self.screen))

        img_path = rospkg.RosPack().get_path('art_projected_gui') + '/imgs'
        self.checkerboard_img = QtGui.QPixmap(img_path + "/pattern.png")
        self.calibrating = False

        desktop = QtGui.QDesktopWidget()
        geometry = desktop.screenGeometry(self.screen)
        self.move(geometry.left(), geometry.top())
        self.resize(geometry.width(), geometry.height())

        self.tfl = None
        self.bridge = CvBridge()

        self.setAutoFillBackground(True)
        p = self.palette()
        p.setColor(self.backgroundRole(), QtCore.Qt.black)
        # self.setPalette(p)

        self.pix_label = QtGui.QLabel(self)
        self.pix_label.setAlignment(QtCore.Qt.AlignHCenter | QtCore.Qt.AlignVCenter)
        # self.pix_label.setScaledContents(True)
        self.pix_label.resize(self.size())

        self.server = rospy.get_param("~scene_server", "127.0.0.1")
        self.port = rospy.get_param("~scene_server_port", 1234)
        self.tcpSocket = QtNetwork.QTcpSocket(self)
        self.blockSize = 0
        self.tcpSocket.readyRead.connect(self.getScene)
        # self.tcpSocket.error.connect(self.displayError)
        self.tcpSocket.connectToHost(self.server, self.port)

        r = rospy.Rate(1.0 / 5)

        while not self.tcpSocket.waitForConnected(1):

            if rospy.is_shutdown():
                return
            rospy.loginfo("Waiting for scene server...")
            self.tcpSocket.connectToHost(self.server, self.port)
            r.sleep()

        rospy.loginfo('Connected to scene server.')

        self.calibrated_pub = rospy.Publisher("/art/interface/projected_gui/projector/" + self.proj_id + "/calibrated", Bool, queue_size=1, latch=True)
        self.calibrated_pub.publish(self.is_calibrated())

        self.srv_calibrate = rospy.Service("/art/interface/projected_gui/projector/" + self.proj_id + "/calibrate", Empty, self.calibrate_srv_cb)
        self.corners_pub = rospy.Publisher("/art/interface/projected_gui/projector/" + self.proj_id + "/corners", PoseArray, queue_size=10, latch=True)

        QtCore.QObject.connect(self, QtCore.SIGNAL('show_chessboard'), self.show_chessboard_evt)

        self.showFullScreen()

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

            if self.tcpSocket.bytesAvailable() > 0:
                rospy.logdebug("Image dropped")
                continue

            ba = QtCore.qUncompress(ba)
            if not pix.loadFromData(ba):

                rospy.logerr("Failed to load image from received data")

            if not self.is_calibrated() or self.calibrating:
                return

            img = pix.convertToFormat(QtGui.QImage.Format_ARGB32)
            v = qimage2ndarray.rgb_view(img)

            # TODO gpu
            image_np = cv2.warpPerspective(v, self.h_matrix, (self.width(), self.height()))  # ,  flags = cv2.INTER_LINEAR

            height, width, channel = image_np.shape
            bytesPerLine = 3 * width
            image = QtGui.QPixmap.fromImage(QtGui.QImage(image_np.data, width, height, bytesPerLine, QtGui.QImage.Format_RGB888))

            self.pix_label.setPixmap(image)
            self.update()
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

        ret, corners = cv2.findChessboardCorners(cv_img, (9, 6), None, flags=cv2.CALIB_CB_ADAPTIVE_THRESH | cv2.CALIB_CB_FILTER_QUADS | cv2.CALIB_CB_NORMALIZE_IMAGE)

        if not ret:

            rospy.logerr("Could not find chessboard corners")
            return False

        criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 100, 0.001)
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

        h, status = cv2.findHomography(np.array(points), np.array(ppoints), cv2.LMEDS)

        self.h_matrix = np.matrix(h)
        # self.h_matrix = np.matrix([[1,  0,  0], [0,  1,  0], [0,  0, 1.0]])

        # store homography matrix to parameter server
        s = str(self.h_matrix.tolist())
        rospy.set_param("~calibration_matrix", s)
        print s

        return True

    def shutdown_ts(self):

        self.timeout_timer.shutdown()

        # TODO is this correct way how to shut it down?
        for sub in self.subs:
            sub.sub.unregister()
        self.ts = None

    def sync_cb(self, image, cam_info, depth):

        self.timeout_timer.shutdown()
        self.timeout_timer = rospy.Timer(rospy.Duration(3.0), self.timeout_timer_cb, oneshot=True)

        if self.calibrate(image, cam_info, depth):

            rospy.loginfo('Calibrated')

        else:

            self.calibration_attempts += 1

            if self.calibration_attempts < 10:
                return

            rospy.logerr('Calibration failed')

        self.shutdown_ts()
        if self.is_calibrated():
            self.tfl = None
        self.calibrated_pub.publish(self.is_calibrated())
        self.calibrating = False

    def show_chessboard_evt(self):

        rat = 1.0  # TODO make checkerboard smaller and smaller if it cannot be detected
        self.pix_label.setPixmap(self.checkerboard_img.scaled(rat * self.width(), rat * self.height(), QtCore.Qt.KeepAspectRatio))

    def timeout_timer_cb(self, evt):

        rospy.logerr("Timeout - no message arrived.")
        self.shutdown_ts()
        self.calibrated_pub.publish(self.is_calibrated())
        self.calibrating = False

    def tfl_delay_timer_cb(self, evt=None):

        rospy.loginfo('Subscribing to camera topics')

        self.subs = []
        self.subs.append(message_filters.Subscriber(self.camera_image_topic, Image))
        self.subs.append(message_filters.Subscriber(self.camera_info_topic, CameraInfo))
        self.subs.append(message_filters.Subscriber(self.camera_depth_topic, Image))

        self.ts = message_filters.TimeSynchronizer(self.subs, 10)
        self.ts.registerCallback(self.sync_cb)

        self.timeout_timer = rospy.Timer(rospy.Duration(3.0), self.timeout_timer_cb, oneshot=True)

    def calibrate_srv_cb(self, req):

        if self.calibrating:
            rospy.logwarn('Calibration already running')
            return None

        rospy.loginfo('Starting calibration')
        self.emit(QtCore.SIGNAL('show_chessboard'))
        self.calibrating = True

        self.calibration_attempts = 0

        # TF Listener needs some time to buffer data
        if self.tfl is None:
            self.tfl = tf.TransformListener()
            self.tfl_timer = rospy.Timer(rospy.Duration(3.0), self.tfl_delay_timer_cb, oneshot=True)
        else:
            self.tfl_delay_timer_cb()

        return EmptyResponse()

    def is_calibrated(self):

        return self.h_matrix is not None

    def on_resize(self, event):

        self.pix_label.resize(self.size())
