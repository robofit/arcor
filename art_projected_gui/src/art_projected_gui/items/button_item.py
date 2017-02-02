#!/usr/bin/env python

from PyQt4 import QtGui, QtCore
from item import Item

# TODO icons
translate = QtCore.QCoreApplication.translate


class ButtonItem(Item):

    def __init__(self, scene, rpm, x, y, caption, parent, clicked, scale=1.0, background_color=QtCore.Qt.green):

        self.background_color = background_color
        self.scale = scale
        self.clicked = clicked
        self.caption = caption
        self.width = None

        super(ButtonItem, self).__init__(scene, rpm, x, y, parent)
        self.setCacheMode(QtGui.QGraphicsItem.ItemCoordinateCache)
        self.setZValue(100)

    def boundingRect(self):

        font = QtGui.QFont('Arial', self.get_font_size(self.scale))
        metrics = QtGui.QFontMetrics(font)

        if self.width is not None:
            w = self.width
        else:
            w = metrics.width(self.caption) + 20 * self.scale
        h = metrics.height() + 20 * self.scale

        return QtCore.QRectF(-1.5, -1.5, w+3,  h+3)

    def cursor_click(self):

        if self.clicked is not None and self.isEnabled():
            self.clicked()

    def set_width(self,  w=None):

        self.width = w
        self.update()

    def set_caption(self, txt):

        self.caption = txt
        self.update()

    def paint(self, painter, option, widget):

        painter.setClipRect(option.exposedRect)
        painter.setRenderHint(QtGui.QPainter.Antialiasing)

        pen = QtGui.QPen()
        pen.setWidth(3)
        if not self.hover:
            pen.setStyle(QtCore.Qt.NoPen)
        pen.setColor(QtCore.Qt.white)
        painter.setPen(pen)

        rect = QtCore.QRectF(1.5, 1.5, self.boundingRect().width()-1.5, self.boundingRect().height()-1.5)

        font = QtGui.QFont('Arial', self.get_font_size(self.scale))
        painter.setFont(font)

        if not self.isEnabled():
            painter.setBrush(QtCore.Qt.gray)
        else:
            painter.setBrush(self.background_color)

        painter.drawRoundedRect(rect, 10.0 * self.scale, 10.0 * self.scale)

        pen.setStyle(QtCore.Qt.SolidLine)
        painter.setPen(pen)

        text_rect = QtCore.QRectF(5 * self.scale+1.5, +1.5, self.boundingRect().width()-5 * self.scale-1.5, self.boundingRect().height()-1.5)
        painter.drawText(text_rect, QtCore.Qt.AlignCenter, self.caption)
