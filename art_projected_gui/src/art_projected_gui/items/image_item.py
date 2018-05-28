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
            h):

        self.img = None
        self.w = w
        self.h = h

        super(ImageItem, self).__init__(scene, x, y)
        self.setCacheMode(QtGui.QGraphicsItem.ItemCoordinateCache)
        self.setZValue(100)

        self.setFlag(QtGui.QGraphicsItem.ItemIsMovable, True)
        self.setFlag(QtGui.QGraphicsItem.ItemIsSelectable, True)

    def set_image(self, img):

        self.img = img.scaled(self.boundingRect().width(),
                              self.boundingRect().height(),
                              QtCore.Qt.KeepAspectRatio | QtCore.Qt.SmoothTransformation)

        self.update()

    def boundingRect(self):

        return QtCore.QRectF(0, 0, self.w, self.h)

    def paint(self, painter, option, widget):

        if not self.scene():
            return

        painter.setClipRect(option.exposedRect)
        painter.setRenderHint(QtGui.QPainter.Antialiasing)

        painter.drawImage(0, 0, self.img)
