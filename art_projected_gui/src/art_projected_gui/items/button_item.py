#!/usr/bin/env python

from PyQt4 import QtGui, QtCore
from item import Item

translate = QtCore.QCoreApplication.translate


class ButtonItem(Item):

    def __init__(self, scene, x, y, caption, parent, clicked, scale=1.0, background_color=QtCore.Qt.green, width=None, push_button=False, image_path=None):

        self.background_color = background_color
        self.scale = scale
        self.clicked = clicked
        self.push_button = push_button
        self.pressed = False

        self.img = None

        self.w = 0
        self.h = 0

        super(ButtonItem, self).__init__(scene, x, y, parent=parent)
        self.setCacheMode(QtGui.QGraphicsItem.ItemCoordinateCache)
        self.setZValue(100)

        self.set_caption(caption)
        self.set_width(width)

        if image_path is not None:

            self.img = QtGui.QImage()
            self.img.load(image_path)
            self.img = self.img.scaled(self.boundingRect().width() * 0.9, self.boundingRect(
            ).height() * 0.9, QtCore.Qt.KeepAspectRatio | QtCore.Qt.SmoothTransformation)

        self.setFlag(QtGui.QGraphicsItem.ItemIsSelectable, True)

    def boundingRect(self):

        return QtCore.QRectF(-1.5, -1.5, self.w + 3, self.h + 3)

    def cursor_click(self):

        if self.isEnabled():

            if self.push_button:

                self.pressed = not self.pressed

            if self.clicked is not None:
                self.clicked(self)

    def set_pressed(self, state):

        self.pressed = state
        self.update()

    def set_width(self, width):

        font = QtGui.QFont('Arial', self.get_font_size(self.scale))
        metrics = QtGui.QFontMetrics(font)

        if width is None:

            self.w = metrics.width(self.caption) + 20 * self.scale
            self.h = metrics.height() + 20 * self.scale

        else:

            br = metrics.boundingRect(QtCore.QRectF(0, 0, self.m2pix(width) - (20 * self.scale), 10000).toRect(
            ), QtCore.Qt.AlignLeft | QtCore.Qt.AlignTop | QtCore.Qt.TextWordWrap, self.caption)

            self.w = max(br.width() + 20 * self.scale, self.m2pix(width))
            self.h = br.height() + 20 * self.scale

        self.update()

    def set_background_color(self, color=QtCore.Qt.green):

        self.background_color = color
        self.update()

    def set_caption(self, caption, width=None):

        self.caption = caption
        if width is not None:
            self.set_width(width)
        self.update()

    def paint(self, painter, option, widget):

        if not self.scene():
            return

        painter.setClipRect(option.exposedRect)
        painter.setRenderHint(QtGui.QPainter.Antialiasing)

        pen = QtGui.QPen()
        pen.setWidth(3)

        if self.hover or (self.push_button and self.pressed):

            pen.setColor(QtCore.Qt.white)

        else:

            pen.setStyle(QtCore.Qt.NoPen)

        painter.setPen(pen)

        rect = QtCore.QRectF(0, 0, self.w, self.h)

        font = QtGui.QFont('Arial', self.get_font_size(self.scale))
        painter.setFont(font)

        if not self.isEnabled():
            painter.setBrush(QtCore.Qt.gray)
        else:
            painter.setBrush(self.background_color)

        painter.drawRoundedRect(rect, 10.0 * self.scale, 10.0 * self.scale)

        pen.setStyle(QtCore.Qt.SolidLine)
        painter.setPen(pen)

        if self.img is not None:

            painter.drawImage(QtCore.QRectF((self.w - self.img.width()) / 2, (self.h -
                                                                              self.img.height()) / 2, self.img.width(), self.img.height()), self.img)

        else:

            text_rect = QtCore.QRectF(
                10 * self.scale, 10 * self.scale, self.w - 20 * self.scale, self.h - 20 * self.scale)

            if '\n' not in self.caption:

                painter.drawText(
                    text_rect, QtCore.Qt.AlignCenter, self.caption)

            else:

                painter.drawText(text_rect, QtCore.Qt.AlignLeft |
                                 QtCore.Qt.AlignTop | QtCore.Qt.TextWordWrap, self.caption)
