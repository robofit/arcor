#!/usr/bin/env python

"""
Visualization of place on the table
"""

from PyQt4 import QtGui, QtCore
from item import Item
from object_item import ObjectItem
from desc_item import DescItem


class PlaceItem(Item):

    def __init__(self, scene, rpm, caption, x, y, place_pose_changed=None, outline_diameter=0.1, selected=False, fixed=False):

        self.outline_diameter = outline_diameter
        self.caption = caption
        self.in_collision = False

        super(PlaceItem, self).__init__(scene, rpm, x, y)

        self.desc = DescItem(scene, rpm, -self.outline_diameter * 1.3 / 2.0, self.outline_diameter * 1.3 / 2 + 0.01, self)
        self.update_text()

        self.fixed = fixed

        self.place_pose_changed = place_pose_changed

    def hover_changed(self):

        self.update_text()
        self.update()

    def update_text(self):

        desc = []
        desc.append(self.caption)

        if self.hover:

            desc.append(self.get_pos_str())

        self.desc.set_content(desc)

    def cursor_release(self):

        if self.place_pose_changed is not None:
            self.place_pose_changed(self.get_pos())

    def boundingRect(self):

        es = self.m2pix(self.outline_diameter * 1.3)
        return QtCore.QRectF(-es / 2, -es / 2, es, es + 40)

    def shape(self):

        path = QtGui.QPainterPath()
        es = self.m2pix(self.outline_diameter)
        path.addEllipse(QtCore.QPoint(0, 0), es / 2, es / 2)
        return path

    def item_moved(self):

        # TODO testovat kolize jen s PlaceItem?
        for it in self.collidingItems():
            if isinstance(it, PlaceItem) or isinstance(it, ObjectItem):
                self.in_collision = True
                break
        else:
            self.in_collision = False

    def paint(self, painter, option, widget):

        painter.setClipRect(option.exposedRect)
        painter.setRenderHint(QtGui.QPainter.Antialiasing)

        es = self.m2pix(self.outline_diameter)

        if self.hover and not self.fixed:
            painter.setBrush(QtCore.Qt.gray)
            painter.setPen(QtCore.Qt.gray)
            painter.drawEllipse(QtCore.QPoint(0, 0), es / 2 * 1.3, es / 2 * 1.3)

        if self.fixed:
            painter.setBrush(QtCore.Qt.gray)
        elif not self.in_collision:
            painter.setBrush(QtCore.Qt.cyan)
        else:
            painter.setBrush(QtCore.Qt.red)

        painter.drawEllipse(QtCore.QPoint(0, 0), es / 2, es / 2)
