#!/usr/bin/env python

from PyQt4 import QtGui, QtCore, QtSvg
from item import Item
import rospy


class IconItem(Item):

    def __init__(
            self,
            scene,
            x,
            y,
            w,
            h,
            fn,
            fixed=False):

        self.img = None
        self.w = 0
        self.h = 0

        super(IconItem, self).__init__(scene, x, y)

        self.icon = QtSvg.QGraphicsSvgItem(fn, self)

        self.setCacheMode(QtGui.QGraphicsItem.ItemCoordinateCache)
        self.setZValue(100)

        self.fixed = fixed
        self.setFlag(QtGui.QGraphicsItem.ItemIsMovable, True)
        self.setFlag(QtGui.QGraphicsItem.ItemIsSelectable, True)

        self.w = self.m2pix(w)
        self.h = self.m2pix(h)

        self.icon.setScale(min(self.boundingRect().height() / self.icon.boundingRect().height(),
                               self.boundingRect().width() / self.icon.boundingRect().width()))

    def boundingRect(self):

        return QtCore.QRectF(0, 0, self.w, self.h)

    def paint(self, painter, option, widget):

        pass
