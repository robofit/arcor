#!/usr/bin/env python

"""
Visualization of place on the table
"""

from PyQt4 import QtGui, QtCore
from item import Item
from object_item import ObjectItem

class PlaceItem(Item):

    def __init__(self,  scene,  rpm, caption,  x,  y,  place_pose_changed=None,  outline_diameter=0.1,  selected = False,  fixed=False):

        self.outline_diameter = outline_diameter
        self.caption = caption
        self.in_collision = False

        super(PlaceItem,  self).__init__(scene,  rpm,  x,  y)

        self.fixed = fixed

        self.place_pose_changed = place_pose_changed

    def cursor_release(self):

        if self.place_pose_changed is not None: self.place_pose_changed(self.get_pos())

    def boundingRect(self):

        es = self.m2pix(self.outline_diameter)
        return QtCore.QRectF(-es/2, -es/2, es, es)

    def item_moved(self):

        # TODO testovat kolize jen s PlaceItem?
        for it in self.collidingItems():
            if isinstance(it,  PlaceItem) or isinstance(it,  ObjectItem):
                self.in_collision = True
                break
        else:
            self.in_collision = False

    def paint(self, painter, option, widget):

        painter.setRenderHint(QtGui.QPainter.Antialiasing)

        es = self.m2pix(self.outline_diameter)

        if self.hover and not self.fixed:
            painter.setBrush(QtCore.Qt.gray)
            painter.setPen(QtCore.Qt.gray)
            painter.drawEllipse(QtCore.QPoint(0,  0), es/2*1.3, es/2*1.3)

        if self.fixed:
            painter.setBrush(QtCore.Qt.gray)
        elif not self.in_collision:
            painter.setBrush(QtCore.Qt.cyan)
        else:
            painter.setBrush(QtCore.Qt.red)

        painter.drawEllipse(QtCore.QPoint(0,  0), es/2, es/2)

        painter.setFont(QtGui.QFont('Arial', 12));

        painter.setPen(QtCore.Qt.gray)

        if self.hover:

            painter.setPen(QtCore.Qt.white)
            painter.drawText(-es/2,  es/2+40, self.get_pos_str())

        painter.drawText(-es/2,  es/2+20, self.caption);
