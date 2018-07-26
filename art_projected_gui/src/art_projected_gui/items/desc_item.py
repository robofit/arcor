#!/usr/bin/env python

from PyQt4 import QtGui, QtCore
from item import Item


class DescItem(Item):

    def __init__(self, scene, x, y, parent):

        self.text = ""
        self.scale = 1.0
        self.color = QtCore.Qt.gray
        self.hover_color = QtCore.Qt.white

        super(DescItem, self).__init__(scene, x, y, parent=parent)

    def boundingRect(self):

        if not self.scene():
            return QtCore.QRectF()

        font = QtGui.QFont(self.default_font, self.get_font_size(self.scale))
        metrics = QtGui.QFontMetrics(font)

        return QtCore.QRectF(
            metrics.boundingRect(
                QtCore.QRect(
                    0,
                    0,
                    10000,
                    10000),
                QtCore.Qt.AlignLeft | QtCore.Qt.AlignTop | QtCore.Qt.TextWordWrap,
                self.text))

    def set_content(self, text, scale=1.0, color=None, hover_color=None):

        self.prepareGeometryChange()
        self.scale = scale
        self.text = text
        if color:
            self.color = color
        if hover_color:
            self.hover_color = hover_color
        self.update()

    def paint(self, painter, option, widget):

        if not self.scene():
            return

        painter.setClipRect(option.exposedRect)
        painter.setRenderHint(QtGui.QPainter.Antialiasing)

        if self.parentItem() and self.parentItem().hover:

            painter.setBrush(self.hover_color)
            painter.setPen(self.hover_color)

        else:

            painter.setBrush(self.color)
            painter.setPen(self.color)

        font = QtGui.QFont(self.default_font, self.get_font_size(self.scale))
        painter.setFont(font)
        painter.drawText(self.boundingRect(), QtCore.Qt.AlignLeft |
                         QtCore.Qt.AlignTop | QtCore.Qt.TextWordWrap, self.text)
