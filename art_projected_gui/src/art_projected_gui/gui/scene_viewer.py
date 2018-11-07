#!/usr/bin/env python

from PyQt4 import QtGui, QtCore, QtNetwork
import rospy


class SceneViewer(QtGui.QWidget):

    def __init__(self):

        self.kill_now = False

        self.pix_label = None

        super(SceneViewer, self).__init__()

        self.ns = "/art/interface/projected_gui/"

        self.server = rospy.get_param(self.ns + "scene_server")
        self.port = rospy.get_param(self.ns + "scene_server_port")
        rospy.loginfo("Server: " + self.server + ":" + str(self.port))

        self.show()

        self.pix_label = QtGui.QLabel(self)
        self.pix_label.setAlignment(
            QtCore.Qt.AlignHCenter | QtCore.Qt.AlignVCenter)

        self.pix_label.resize(self.size())
        self.pix_label.show()

        self.tcpSocket = QtNetwork.QTcpSocket(self)
        self.blockSize = 0
        self.tcpSocket.readyRead.connect(self.getScene)
        self.tcpSocket.error.connect(self.on_error)

        rospy.loginfo("Scene viewer ready")

    def keyPressEvent(self, e):

        if e.key() == QtCore.Qt.Key_Escape:
            self.close()

    def connect(self):

        r = rospy.Rate(1.0 / 5)

        while not self.tcpSocket.waitForConnected(1) and not self.kill_now:

            rospy.loginfo("Waiting for scene server...")
            self.tcpSocket.connectToHost(self.server, self.port)
            r.sleep()

        if not self.kill_now:
            rospy.loginfo('Connected to scene server.')

    def on_error(self):

        rospy.logerr("socket error")
        if self.kill_now:
            self.tcpSocket.close()
            return
        QtCore.QTimer.singleShot(0, self.connect)

    def getScene(self):

        instr = QtCore.QDataStream(self.tcpSocket)
        instr.setVersion(QtCore.QDataStream.Qt_4_0)

        while not self.kill_now:

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
                rospy.loginfo("Frame dropped")
                continue

            # 16ms
            if not pix.loadFromData(ba, "JPG"):
                rospy.logerr("Failed to load image from received data")
                return

            self.get_image(pix)

    def get_image(self, pix):

        pix = pix.mirrored(vertical=True)
        image = QtGui.QPixmap.fromImage(
            pix.scaled(
                self.pix_label.width(),
                self.pix_label.height(),
                QtCore.Qt.KeepAspectRatio,
                transformMode=QtCore.Qt.SmoothTransformation))
        self.pix_label.setPixmap(image)
        self.update()

    def resizeEvent(self, event):

        if self.pix_label:
            self.pix_label.resize(self.size())
