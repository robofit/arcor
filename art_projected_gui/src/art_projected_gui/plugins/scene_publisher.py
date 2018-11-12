from art_projected_gui.plugins import GuiPlugin
import rospy
from PyQt4 import QtCore, QtNetwork, QtGui

translate = QtCore.QCoreApplication.translate


class ScenePublisherPlugin(GuiPlugin):

    def __init__(self, ui, parameters):

        super(ScenePublisherPlugin, self).__init__(ui)

        self.port = 1234  # TODO read from param

        self.tcpServer = QtNetwork.QTcpServer(self)
        if not self.tcpServer.listen(port=self.port):
            rospy.logerr(
                'Failed to start scene TCP server on port ' + str(self.port))

        self.tcpServer.newConnection.connect(self.new_connection)
        self.connections = []

        self.scene_timer = QtCore.QTimer()
        self.connect(
            self.scene_timer,
            QtCore.SIGNAL('timeout()'),
            self.send_to_clients_evt)
        self.scene_timer.start(1.0 / 15 * 1000)

    def new_connection(self):

        rospy.loginfo('Some projector node just connected.')
        self.connections.append(self.tcpServer.nextPendingConnection())
        self.connections[-1].setSocketOption(
            QtNetwork.QAbstractSocket.LowDelayOption, 1)

        # TODO deal with disconnected clients!
        # self.connections[-1].disconnected.connect(clientConnection.deleteLater)

    def send_to_clients_evt(self):

        if len(self.connections) == 0:
            return

        # start = time.time()

        pix = QtGui.QImage(
            self.ui.scene.width(),
            self.ui.scene.height(),
            QtGui.QImage.Format_RGB888)
        painter = QtGui.QPainter(pix)
        painter.setRenderHint(QtGui.QPainter.Antialiasing)
        self.ui.scene.render(painter)
        painter.end()
        pix = pix.mirrored()

        block = QtCore.QByteArray()
        out = QtCore.QDataStream(block, QtCore.QIODevice.WriteOnly)
        out.setVersion(QtCore.QDataStream.Qt_4_0)
        out.writeUInt32(0)

        img = QtCore.QByteArray()
        buffer = QtCore.QBuffer(img)
        buffer.open(QtCore.QIODevice.WriteOnly)
        pix.save(buffer, "JPG", 95)
        out << img

        out.device().seek(0)
        out.writeUInt32(block.size() - 4)

        # print block.size()

        for con in self.connections:

            con.write(block)

        # end = time.time()
        # rospy.logdebug("Image sent in: " + str(end-start))
