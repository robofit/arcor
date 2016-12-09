#!/usr/bin/env python

"""
Visualization of detected object(s).

TODO:
 - preselect (highlight)
 - display additional information on highlight
 - diameter based on boundingbox size?

"""

from PyQt4 import QtGui, QtCore
from item import Item
from desc_item import DescItem

translate = QtCore.QCoreApplication.translate


class ObjectItem(Item):

    def __init__(self, scene, rpm, object_id, object_type, x, y, sel_cb=None, outline_diameter=0.1, selected=False):

        self.object_id = object_id
        self.object_type = object_type
        self.outline_diameter = outline_diameter
        self.selected = selected
        self.sel_cb = sel_cb

        super(ObjectItem, self).__init__(scene, rpm, x, y)

        self.desc = DescItem(scene, rpm, -self.outline_diameter * 1.3 / 2.0, self.outline_diameter * 1.3 / 2 + 0.01, self)
        self.update_text()

        if selected:
            self.set_selected()

    def update_text(self):

        desc = []
        desc.append(translate("ObjectItem", "ID: ") + self.object_id)

        if self.hover:

            desc.append(translate("ObjectItem", "TYPE: ") + self.object_type)
            desc.append(self.get_pos_str())

        self.desc.set_content(desc)

    def hover_changed(self):

        self.update_text()
        self.update()

    def boundingRect(self):

        es = self.m2pix(self.outline_diameter * 1.3)
        p = 1.0
        return QtCore.QRectF(-es / 2 - p, -es / 2 - p, es + 2 * p, es + 2 * p)

    def shape(self):

        path = QtGui.QPainterPath()
        es = self.m2pix(self.outline_diameter)
        path.addEllipse(QtCore.QPoint(0, 0), es / 2, es / 2)
        return path

    def paint(self, painter, option, widget):

        painter.setClipRect(option.exposedRect)
        painter.setRenderHint(QtGui.QPainter.Antialiasing)

        eso = self.m2pix(self.outline_diameter * 1.3)
        es = self.m2pix(self.outline_diameter)

        if self.selected:

            painter.setBrush(QtCore.Qt.green)
            painter.setPen(QtCore.Qt.green)

            painter.drawEllipse(QtCore.QPoint(0, 0), eso / 2, eso / 2)

        elif self.hover:

            painter.setBrush(QtCore.Qt.gray)
            painter.setPen(QtCore.Qt.gray)

            painter.drawEllipse(QtCore.QPoint(0, 0), eso / 2, eso / 2)
            # TODO disp add info

        painter.setBrush(QtCore.Qt.white)
        painter.setPen(QtCore.Qt.white)

        painter.drawEllipse(QtCore.QPoint(0, 0), es / 2, es / 2)

    def cursor_press(self):  # TODO cursor_click??

        if self.sel_cb is not None:
            # callback should handle object selection
            self.sel_cb(self.object_id, self.selected)

        else:
            # no callback - object will handle its selection
            if not self.selected:
                self.set_selected()
            else:
                self.set_selected(False)

    def set_selected(self, selected=True):

        if selected:

            self.selected = True
            # rospy.logdebug('Object ID ' + self.object_id + ' selected')

        else:

            self.selected = False
            # rospy.logdebug('Object ID ' + self.object_id + ' unselected')

        self.update()
