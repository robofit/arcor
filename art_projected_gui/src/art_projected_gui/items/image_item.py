#!/usr/bin/env python

from PyQt4 import QtGui, QtCore
from item import Item

# TODO label over image (semi-transparent overlay?)


class ImageItem(Item):

    def __init__(
            self,
            scene,
            x,
            y,
            w,
            h,
            fixed=False):

        self.img = None
        self.w = 0
        self.h = 0
        self._text = None
        self._text_color = None

        super(ImageItem, self).__init__(scene, x, y)
        self.setCacheMode(QtGui.QGraphicsItem.ItemCoordinateCache)
        self.setZValue(100)

        self.fixed = fixed
        self.setFlag(QtGui.QGraphicsItem.ItemIsMovable, True)
        self.setFlag(QtGui.QGraphicsItem.ItemIsSelectable, True)

        self.w = self.m2pix(w)
        self.h = self.m2pix(h)

    def set_image(self, img):

        self.img = img.scaled(self.boundingRect().width(),
                              self.boundingRect().height(),
                              QtCore.Qt.KeepAspectRatio | QtCore.Qt.SmoothTransformation)

        self.update()

    def set_text(self, txt=None, color=None):

        self._text = txt
        self._text_color = color

    def boundingRect(self):

        return QtCore.QRectF(0, 0, self.w, self.h)

    def paint(self, painter, option, widget):

        if not self.scene() or not self.img:
            return

        painter.setClipRect(option.exposedRect)
        painter.setRenderHint(QtGui.QPainter.Antialiasing)

        painter.drawImage(0, 0, self.img)

        # TODO fix font size - make it fit whole item
        font = QtGui.QFont('Arial', 70)
        painter.setFont(font)

        if self._text:

            painter.setPen(self._text_color)
            painter.drawText(QtCore.QRectF(0.0, 0.0, self.w, self.h),
                             QtCore.Qt.AlignHCenter | QtCore.Qt.AlignVCenter, self._text)
