#!/usr/bin/env python

""" Visualization of point(s) to be touched during touchtable calibration. """

from PyQt4 import QtGui, QtCore
from item import Item

translate = QtCore.QCoreApplication.translate


class TouchPointsItem(Item):

    def __init__(self, scene, points=[], outline_diameter=0.01):

        self.outline_diameter = outline_diameter
        self.points = points
        self.current_point_idx = 0
        super(TouchPointsItem, self).__init__(scene, self.points[self.current_point_idx][0], self.points[self.current_point_idx][1])

    def boundingRect(self):

        if not self.scene():
            return QtCore.QRectF()

        es = self.m2pix(self.outline_diameter * 6) + 2 * self.m2pix(self.outline_diameter * 0.2)
        return QtCore.QRectF(-es / 2, -es / 2, es, es)

    def next(self):

        if self.current_point_idx < len(self.points) - 1:

            self.current_point_idx += 1
            self.set_pos(self.points[self.current_point_idx][0], self.points[self.current_point_idx][1])
            self.update()
            return True

        else:

            return False

    def paint(self, painter, option, widget):

        if not self.scene():
            return

        painter.setClipRect(option.exposedRect)
        painter.setRenderHint(QtGui.QPainter.Antialiasing)

        es = self.m2pix(self.outline_diameter)

        pen = QtGui.QPen()
        pen.setColor(QtCore.Qt.white)
        pen.setWidth(self.m2pix(self.outline_diameter * 0.2))
        painter.setPen(pen)

        painter.setBrush(QtCore.Qt.NoBrush)

        for i in range(5, 0, -2):

            pix = self.m2pix(self.outline_diameter * i)

            painter.drawEllipse(QtCore.QPoint(0, 0), (es + pix) / 2, (es + pix) / 2)

        painter.setBrush(QtCore.Qt.white)
        painter.setPen(QtCore.Qt.white)

        pen.setWidth(0)
        painter.setPen(pen)

        painter.drawEllipse(QtCore.QPoint(0, 0), es / 2, es / 2)
