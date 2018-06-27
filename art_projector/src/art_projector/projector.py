#!/usr/bin/env python

from PyQt4 import QtGui, QtCore
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
from art_utils import array_from_param
from art_projected_gui.gui import SceneViewer
import pickle
import rospkg
import os


class Padding(object):

    def __init__(self):

        self.top = rospy.get_param(
            '~padding/top', 0.0)

        self.bottom = rospy.get_param(
            '~padding/bottom', 0.0)

        self.left = rospy.get_param(
            '~padding/left', 0.0)

        self.right = rospy.get_param(
            '~padding/right', 0.0)

    @property
    def width(self):

        return self.left + self.right

    @property
    def height(self):

        return self.top + self.bottom


class Projector(SceneViewer):

    def __init__(self):

        super(Projector, self).__init__()

        while True:
            try:
                self.world_frame = rospy.get_param('/art/conf/world_frame')
                break
            except KeyError:
                rospy.loginfo("Waiting for global parameters...")
                rospy.sleep(1.0)

        self.proj_id = rospy.get_param('~projector_id', 'test')
        self.screen = rospy.get_param('~screen_number', 0)
        self.camera_image_topic = rospy.get_param(
            '~camera_image_topic', 'kinect2/hd/image_color_rect')
        self.camera_depth_topic = rospy.get_param(
            '~camera_depth_topic', 'kinect2/hd/image_depth_rect')
        self.camera_info_topic = rospy.get_param(
            '~camera_info_topic', 'kinect2/hd/camera_info')

        # padding serves to restrict area usable for calibration (flat surface)
        self.padding = Padding()

        self.map_x = None
        self.map_y = None

        self.dx = None
        self.dy = None
        self.scaled_checkerboard_width = None

        self.rpm = int(rospy.get_param(self.ns + 'rpm'))

        self.scene_size = array_from_param(self.ns + "scene_size", float, 2)
        self.scene_origin = array_from_param(self.ns + "scene_origin", float, 2)

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

        self.projectors_calibrated_sub = rospy.Subscriber(
            '/art/interface/projected_gui/app/projectors_calibrated', Bool, self.projectors_calibrated_cb,
            queue_size=10)
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

        rospack = rospkg.RosPack()
        self.map_path = os.path.join(rospack.get_path('art_projector'), 'data', self.proj_id + '.ptf')

        if h_matrix is not None:
            rospy.loginfo('Loaded calibration from param.')
            self.calibrated = True
            self.calibrated_pub.publish(self.is_calibrated())

            try:
                with open(self.map_path, 'rb') as f:

                    self.map_x, self.map_y = pickle.load(f)
                    self.maps_ready = True
                    rospy.loginfo("Map loaded from file")

            except (IOError, OSError) as e:  # TODO pickle exceptions?
                rospy.logwarn("Failed to load map from file")
                pass

            if not self.maps_ready:
                self.init_map_from_matrix(np.matrix(ast.literal_eval(h_matrix)))
        else:

            try:
                os.remove(self.map_path)
            except (IOError, OSError):
                pass

            self.calibrated_pub.publish(self.is_calibrated())

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

        try:
            with open(self.map_path, 'wb') as f:

                pickle.dump((self.map_x, self.map_y), f)

        except (IOError, OSError) as e:
            rospy.logerr("Failed to store map to file: " + str(e))
            pass

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

    def get_image(self, pix):

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
            cv_img, (9, 6), None,
            flags=cv2.CALIB_CB_ADAPTIVE_THRESH | cv2.CALIB_CB_FILTER_QUADS | cv2.CALIB_CB_NORMALIZE_IMAGE)

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

        box_size = self.scaled_checkerboard_width / 12.0

        # TODO self.scene_origin ???
        # generate requested table coordinates
        for y in range(0, 6):
            for x in range(0, 9):
                px = 2 * box_size + x * box_size + self.dx
                py = 2 * box_size + y * box_size + self.dy

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

        self.pix_label.show()

        pix = QtGui.QPixmap(self.width(), self.height())
        painter = QtGui.QPainter(pix)

        scaled_img = self.checkerboard_img.scaled(
            self.width() - self.padding.width,
            self.height() - self.padding.height, QtCore.Qt.KeepAspectRatio).toImage()

        rospy.logdebug("Checkerboard width: " + str(scaled_img.width()) + ", height: " + str(scaled_img.height()))

        self.dx = self.padding.left + (self.width() - scaled_img.width() - self.padding.width) / 2.0
        self.dy = self.padding.top + (self.height() - scaled_img.height() - self.padding.height) / 2.0

        self.scaled_checkerboard_width = scaled_img.width()

        rospy.logdebug("dx: " + str(self.dx) + ", dy: " + str(self.dy))

        painter.fillRect(pix.rect(), QtGui.QBrush(QtCore.Qt.white))
        painter.drawImage(self.dx, self.dy, scaled_img)
        painter.end()

        self.pix_label.setPixmap(pix)

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
