#!/usr/bin/env python

from PyQt4 import QtGui, QtCore
from item import Item


class PointItem(Item):

    def __init__(self, scene, rpm, x, y, parent, fixed=False):

        self.outline_diameter = 0.025

        super(PointItem, self).__init__(scene, rpm, x, y, parent)
        self.fixed = fixed

    def boundingRect(self):

        es = self.m2pix(self.outline_diameter * 1.8)

        return QtCore.QRectF(-es / 2, -es / 2, es, es)

    def shape(self):

        path = QtGui.QPainterPath()
        es = self.m2pix(self.outline_diameter)
        path.addEllipse(QtCore.QPoint(0, 0), es / 2, es / 2)
        return path

    def item_moved(self):

        self.parentItem().point_changed()  # TODO change it to callback

    def cursor_release(self):

        self.parentItem().point_changed(True)

    def paint(self, painter, option, widget):

        painter.setClipRect(option.exposedRect)
        painter.setRenderHint(QtGui.QPainter.Antialiasing)

        es = self.m2pix(self.outline_diameter)

        if self.hover:
            # TODO coordinates
            painter.setBrush(QtCore.Qt.gray)
            painter.setPen(QtCore.Qt.gray)
            painter.drawEllipse(QtCore.QPoint(0, 0), es / 2 * 1.8, es / 2 * 1.8)

        painter.setBrush(QtCore.Qt.cyan)
        painter.setPen(QtCore.Qt.cyan)

        painter.drawEllipse(QtCore.QPoint(0, 0), es / 2, es / 2)
