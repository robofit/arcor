#!/usr/bin/env python

from PyQt4 import QtGui, QtCore
from item import Item
import rospy
from collections import deque

class LabelItem(Item):

    def __init__(self,  scene,  rpm,  x,  y,  w,  h):

        self.w = w
        self.h = h

        super(LabelItem,  self).__init__(scene,  rpm,  x,  y)

        # TODO list of messages - display them intelligently, optional duration of message etc
        self.msgs = deque()
        self.current_msg = None

    def add_msg(self,  msg,  min_duration=rospy.Duration(3)):

        self.msgs.append({"msg": msg,  "min_duration": min_duration,  "shown_at": None})
        self.update()

    def boundingRect(self):

        w = self.m2pix(self.w)
        h = self.m2pix(self.h)

        return QtCore.QRectF(0,  0, w, h)

    def paint(self, painter, option, widget):

        if (self.current_msg is None or rospy.Time.now() - self.current_msg["shown_at"] > self.current_msg["min_duration"]) and (self) and len(self.msgs) > 0:
                self.current_msg = self.msgs.popleft()
                self.current_msg["shown_at"] = rospy.Time.now()

        if self.current_msg is None: return

        painter.setBrush(QtCore.Qt.white)
        painter.setPen(QtCore.Qt.white)

        # TODO font size
        painter.setFont(QtGui.QFont('Arial', 18))

        painter.drawText(0,  0, self.current_msg["msg"])
