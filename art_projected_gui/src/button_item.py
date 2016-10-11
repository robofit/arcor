#!/usr/bin/env python

from PyQt4 import QtGui, QtCore
from item import Item

# TODO icons
translate = QtCore.QCoreApplication.translate

class ButtonItem(Item):

    def __init__(self,  scene,  rpm,  x,  y,  caption,  parent,  clicked):

        self.w = 180 # TODO spocitat podle rpm
        self.h = 20
        self.clicked = clicked
        self.caption = caption
        self.enabled = False

        super(ButtonItem,  self).__init__(scene,  rpm,  x,  y,  parent)

    def boundingRect(self):

        return QtCore.QRectF(0,  0, self.w, self.h)

    def cursor_click(self):

        if self.clicked is not None and self.enabled: self.clicked()

    def set_caption(self,  txt):

        self.caption = txt
        self.update()

    def paint(self, painter, option, widget):

        pen = QtGui.QPen()
        if not self.hover: pen.setStyle(QtCore.Qt.NoPen)
        pen.setColor(QtCore.Qt.white)
        painter.setPen(pen)

        rect = QtCore.QRectF(5, 0.0, self.w, 40.0)

        font = QtGui.QFont('Arial', 12)
        painter.setFont(font);

        if not self.enabled:
            painter.setBrush(QtCore.Qt.gray)
        else:
            painter.setBrush(QtCore.Qt.green)
        metrics = QtGui.QFontMetrics(font)
        txt = self.caption
        rect.setWidth(metrics.width(txt)+20)

        painter.drawRoundedRect(rect, 10.0, 10.0);

        pen.setStyle(QtCore.Qt.SolidLine)
        painter.setPen(pen)

        painter.drawText(rect,  QtCore.Qt.AlignCenter,  txt)

