#!/usr/bin/env python

from PyQt4 import QtGui, QtCore
from item import Item

# TODO icons
translate = QtCore.QCoreApplication.translate


class ButtonItem(Item):

    def __init__(self, scene, rpm, x, y, caption, parent, clicked, scale=1.0, background_color=QtCore.Qt.green):

        self.background_color = background_color
        self.scale = scale
        self.w = 180 * scale  # TODO spocitat podle rpm
        self.h = 40 * scale
        self.clicked = clicked
        self.caption = caption

        super(ButtonItem, self).__init__(scene, rpm, x, y, parent)
        self.setCacheMode(QtGui.QGraphicsItem.ItemCoordinateCache)
        self.setZValue(100)

    def boundingRect(self):

        return QtCore.QRectF(0, 0, self.w, self.h)

    def cursor_click(self):

        if self.clicked is not None and self.isEnabled():
            self.clicked()

    def set_caption(self, txt):

        self.caption = txt
        self.update()

    def paint(self, painter, option, widget):

        painter.setClipRect(option.exposedRect)
        painter.setRenderHint(QtGui.QPainter.Antialiasing)

        pen = QtGui.QPen()
        if not self.hover:
            pen.setStyle(QtCore.Qt.NoPen)
        pen.setColor(QtCore.Qt.white)
        painter.setPen(pen)

        rect = QtCore.QRectF(5 * self.scale, 0.0, self.w, 40.0 * self.scale)

        font = QtGui.QFont('Arial', self.get_font_size(self.scale))
        painter.setFont(font)

        if not self.isEnabled():
            painter.setBrush(QtCore.Qt.gray)
        else:
            painter.setBrush(self.background_color)
        metrics = QtGui.QFontMetrics(font)
        txt = self.caption
        rect.setWidth(metrics.width(txt) + 20 * self.scale)

        painter.drawRoundedRect(rect, 10.0 * self.scale, 10.0 * self.scale)

        pen.setStyle(QtCore.Qt.SolidLine)
        painter.setPen(pen)

        painter.drawText(rect, QtCore.Qt.AlignCenter, txt)
