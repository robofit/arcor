#!/usr/bin/env python

from PyQt4 import QtGui, QtCore
from item import Item


class DescItem(Item):

    def __init__(self, scene, rpm, x, y, parent):

        self.w = 0
        self.h = 0
        self.lines = []
        self.scale = 1.0
        self.padding = 5  # TODO should be in meters

        super(DescItem, self).__init__(scene, rpm, x, y, parent)

    def boundingRect(self):

        return QtCore.QRectF(0, 0, self.w, self.h)

    def set_content(self, lines, scale=1.0):

        self.prepareGeometryChange()

        self.lines = lines
        self.w = 0
        self.h = 0
        self.scale = scale

        font = QtGui.QFont(self.default_font, self.get_font_size(self.scale))
        metrics = QtGui.QFontMetrics(font)

        self.h = (len(lines) - 1) * (20 * self.scale) + len(lines) * metrics.height()

        for l in lines:

            if metrics.width(l) > self.w:
                self.w = metrics.width(l)

        self.w += 2 * self.padding
        self.h += 2 * self.padding

        self.update()

    def paint(self, painter, option, widget):

        painter.setClipRect(option.exposedRect)
        painter.setRenderHint(QtGui.QPainter.Antialiasing)

        if self.parentItem().hover:

            painter.setBrush(QtCore.Qt.white)
            painter.setPen(QtCore.Qt.white)

        else:

            painter.setBrush(QtCore.Qt.gray)
            painter.setPen(QtCore.Qt.gray)

        font = QtGui.QFont(self.default_font, self.get_font_size(self.scale))
        metrics = QtGui.QFontMetrics(font)
        h = metrics.height()

        for i in range(0, len(self.lines)):

            painter.drawText(self.padding, self.padding + h + i * 20 * self.scale, self.lines[i])
