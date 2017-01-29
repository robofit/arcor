#!/usr/bin/env python

from PyQt4 import QtGui, QtCore,  QtSvg
from item import Item
import rospy
from collections import deque
from art_msgs.srv import NotifyUserRequest
import rospkg


class LabelItem(Item):

    def __init__(self, scene, rpm, x, y, w, h):

        self.w = w
        self.h = h

        super(LabelItem, self).__init__(scene, rpm, x, y)

        # TODO list of messages - display them intelligently, optional duration of message etc
        self.msgs = deque()
        self.current_msg = None

        rospack = rospkg.RosPack()
        self.icons_path = rospack.get_path('art_projected_gui') + '/icons/'

        self.icons = {}
        self.icons[NotifyUserRequest.INFO] = QtSvg.QGraphicsSvgItem(self.icons_path + 'Antu_dialog-information.svg',  self)
        self.icons[NotifyUserRequest.WARN] = QtSvg.QGraphicsSvgItem(self.icons_path + 'Antu_dialog-warning.svg',  self)
        self.icons[NotifyUserRequest.ERROR] = QtSvg.QGraphicsSvgItem(self.icons_path + 'Antu_emblem-important.svg',  self)
        self.icons[NotifyUserRequest.YES_NO_QUESTION] = QtSvg.QGraphicsSvgItem(self.icons_path + 'Antu_dialog-question.svg',  self)

        s = self.m2pix(self.h)
        for k, v in self.icons.iteritems():

            v.setScale(s/v.boundingRect().height())
            v.setPos(0, 0)
            v.setVisible(False)

        self.still_msgs = []
        self.temp_msgs = []

        self.timer = rospy.Timer(rospy.Duration(0.1), self.timer_cb)
        self.setCacheMode(QtGui.QGraphicsItem.ItemCoordinateCache)
        self.setZValue(200)

    def add_msg(self, msg, message_type,  min_duration=rospy.Duration(3), temp=False):

        self.msgs.append({"msg": msg, "min_duration": min_duration, "shown_at": None, "temp": temp,  "type": message_type})
        self.check_for_msgs()

    def boundingRect(self):

        w = self.m2pix(self.w)
        h = self.m2pix(self.h)

        return QtCore.QRectF(0, 0, w, h)

    def prune_old_msgs(self, arr):

        msgs_to_delete = []

        for msg in arr:

            if msg["temp"] is False:  # still messages

                if len(arr) <= 1:  # nothing to do if there is only one message

                    break

                if len(msgs_to_delete) == 0:
                    msgs_to_delete.append(msg)
                else:

                    if msg["shown_at"] is not None and rospy.Time.now() - msg["shown_at"] > msg["min_duration"] and msg["shown_at"] < msgs_to_delete[0]["shown_at"]:

                        msgs_to_delete[0] = msg

            else:

                if msg["shown_at"] is not None and rospy.Time.now() - msg["shown_at"] > msg["min_duration"]:

                    msgs_to_delete.append(msg)

        for msg in msgs_to_delete:

            arr.remove(msg)

        return len(msgs_to_delete) > 0

    def check_for_msgs(self):

        if len(self.msgs) > 0:

            tmp = self.msgs.popleft()

            if tmp["temp"] is False:

                self.still_msgs.append(tmp)

            else:

                self.temp_msgs.append(tmp)

        if len(self.still_msgs) > 0 or len(self.temp_msgs) > 0:
            self.update()

    def timer_cb(self, evt):

        if self.prune_old_msgs(self.still_msgs) or self.prune_old_msgs(self.temp_msgs):
            self.update()

    def paint(self, painter, option, widget):

        msg = None

        if len(self.temp_msgs) > 0:
            msg = self.temp_msgs[0]
        elif len(self.still_msgs) > 0:
            msg = self.still_msgs[0]

        if msg is None:
            return

        if msg["shown_at"] is None:

            msg["shown_at"] = rospy.Time.now()

        for k, v in self.icons.iteritems():

            if k == msg["type"]:
                v.setVisible(True)
            else:
                v.setVisible(False)

        painter.setClipRect(option.exposedRect)
        painter.setBrush(QtCore.Qt.white)
        painter.setPen(QtCore.Qt.white)

        # TODO fix for multiline messages
        font = QtGui.QFont('Arial', self.get_font_size(1.5))
        painter.setFont(font)
        metrics = QtGui.QFontMetrics(font)
        h = self.m2pix(self.h)
        left_padding = h+self.m2pix(0.02)

        painter.drawText(left_padding, (h-metrics.height())/2, self.m2pix(self.w)-left_padding, h, QtCore.Qt.AlignLeft, msg["msg"])
