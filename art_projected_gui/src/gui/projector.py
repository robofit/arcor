#!/usr/bin/env python

from PyQt4 import QtGui, QtCore
import rospkg
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import CompressedImage,  Image
import rospy
import cv2
import numpy as np
from std_msgs.msg import Bool
from std_srvs.srv import Empty, EmptyResponse

# TODO create ProjectorROS (to separate QT / ROS stuff)
# TODO zobrazit "waiting for data" nebo tak neco
# warpovani obrazu pro kazdy z projektoru
# podle vysky v pointcloudu se vymaskuji mista kde je neco vyssiho - aby se promitalo jen na plochu stolu ????

class Projector(QtGui.QWidget):

    def __init__(self, proj_id, screen,  camera_image_topic, camera_info_topic,  calibration):

        super(Projector, self).__init__()

        rospy.loginfo("Projector '" + proj_id + "', on screen " + str(screen) )

        self.proj_id = proj_id

        img_path = rospkg.RosPack().get_path('art_projected_gui') + '/imgs'
        self.checkerboard_img = QtGui.QPixmap(img_path + "/pattern.png")
        self.calibrating = False
        self.camera_image_topic = camera_image_topic
        self.camera_info_topic = camera_info_topic

        desktop = QtGui.QDesktopWidget()
        geometry = desktop.screenGeometry(screen)
        self.move(geometry.left(), geometry.top())
        self.resize(geometry.width(), geometry.height())

        self.bridge = CvBridge()

        self.setAutoFillBackground(True)
        p = self.palette()
        p.setColor(self.backgroundRole(), QtCore.Qt.black)
        #self.setPalette(p)

        self.pix_label = QtGui.QLabel(self)
        self.pix_label.setAlignment(QtCore.Qt.AlignHCenter | QtCore.Qt.AlignVCenter)
        self.pix_label.resize(self.size())

        QtCore.QObject.connect(self, QtCore.SIGNAL('scene'), self.scene_evt)
        self.scene_sub = rospy.Subscriber("/art/interface/projected_gui/scene",  CompressedImage,  self.scene_cb,  queue_size=1)

        self.calibrated = False
        self.calibrated_pub = rospy.Publisher("/art/interface/projected_gui/projector/" + proj_id + "/calibrated",  Bool,  queue_size=1,  latch=True)
        self.calibrated_pub.publish(self.calibrated)

        self.srv_calibrate = rospy.Service("/art/interface/projected_gui/projector/" + proj_id + "/calibrate", Empty, self.calibrate_srv_cb)

        self.showFullScreen()


    def calibrate_srv_cb(self,  req):

        # TODO start calibration ;)
        self.calibrated = True
        self.calibrated_pub.publish(self.calibrated)

        return EmptyResponse()

    def scene_cb(self,  msg):

        if not self.calibrated: return

        np_arr = np.fromstring(msg.data, np.uint8)
        image_np = cv2.imdecode(np_arr, cv2.CV_LOAD_IMAGE_COLOR)

        height, width, channel = image_np.shape
        bytesPerLine = 3 * width
        image = QtGui.QPixmap.fromImage(QtGui.QImage(image_np.data, width, height, bytesPerLine, QtGui.QImage.Format_RGB888))
        self.emit(QtCore.SIGNAL('scene'),  image)

    def scene_evt(self,  img):

        if self.calibrating: return

        # TODO warp image according to calibration
        self.pix_label.setPixmap(img)

    def is_calibrated(self):

        return True

    def on_resize(self, event):

        self.pix_label.resize(self.size())

    def calibrate(self,  cb):

        self.calibrating = True

        # TODO resize image to fit e.g. 75% of screen?
        self.pix_label.setPixmap(self.checkerboard_img)

        # TODO get camera info?

        # subscribe to camera image
        #self.img_sub = rospy.Subscriber(self.camera_topic, Image, self.image_cb)
        img = rospy.wait_for_message(self.camera_image_topic, Image, 1.0)
        try:
                  cv_img = self.bridge.imgmsg_to_cv2(img, "bgr8")
        except CvBridgeError as e:
          print(e)

        cv_img = cv2.cvtColor(cv_img, cv2.COLOR_BGR2GRAY)

        ret, corners = cv2.findChessboardCorners(cv_img, (9,6), None, flags=cv2.CALIB_CB_ADAPTIVE_THRESH | cv2.CALIB_CB_FILTER_QUADS | cv2.CALIB_CB_NORMALIZE_IMAGE)

        if ret == False:

            rospy.logerr("Could not find chessboard corners")
            return

        criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 100, 0.001)
        cv2.cornerSubPix(cv_img,corners,(11,11),(-1,-1),criteria)
        corners = corners.reshape(1,-1,2)[0]

        print corners

        if cb is not None: cb(self)



