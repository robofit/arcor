#!/usr/bin/env python

"""
Visualization of detected object(s).

"""

from PyQt4 import QtGui, QtCore
from item import Item
from desc_item import DescItem
import math
import numpy as np
import tf

translate = QtCore.QCoreApplication.translate


class ObjectItem(Item):

    """The class to visualize (detected) object.

    It currently supports only rotation around z-axis.

    """

    def __init__(self, scene, object_id, object_type, x,
                 y, z, yaw, sel_cb=None, selected=False):

        self.object_id = object_id
        self.selected = selected
        self.sel_cb = sel_cb
        # TODO check bbox type and use rectangle (used now) / ellipse, consider
        # other angles
        self.object_type = object_type
        self.inflate = 0.01
        self.hover_ratio = 1.1
        self.def_color = QtCore.Qt.gray
        self.lx = 0
        self.ly = 0

        self.desc = None
        self.rpy = (0, 0, 0)

        super(ObjectItem, self).__init__(scene, x, y, z)

        self.desc = DescItem(scene, 0, 0, parent=self)
        self.desc.setFlag(QtGui.QGraphicsItem.ItemIgnoresTransformations)

        self.update_text()

        self.setRotation(yaw)

        if selected:
            self.set_selected()

        self._update_desc_pos()
        self.setZValue(50)

    def set_color(self, color=QtCore.Qt.gray):

        self.def_color = color
        self.update()

    def _update_desc_pos(self):

        if self.desc is not None:

            # make upper left corner of description aligned with left extent of
            # the (possibly rotated) object bounding box (highlight area)
            self.desc.setPos(
                self.mapFromScene(
                    self.x() -
                    self.sceneBoundingRect().width() /
                    2,
                    self.y() +
                    self.sceneBoundingRect().height() /
                    2 +
                    self.m2pix(0.01)))

    def set_pos(self, x, y, z, parent_coords=False):

        super(ObjectItem, self).set_pos(x, y, z, parent_coords)
        self._update_desc_pos()
        self.update_text()

        # TODO consider BB size + orientation
        if z > 0.08:
            self.set_enabled(False, True)
        else:
            self.set_enabled(True, True)

    def set_orientation(self, rpy):

        self.rpy = rpy

        # if (self.rpy[0] == 0 or self.rpy[0] == 180) and self.rpy[1] == 0:
        if (abs(self.rpy[0]) < 45 or abs(self.rpy[0]) > 135) and abs(self.rpy[1]) < 45:

            # bez rotace kolem osy X nebo Y (nastojato)
            self.lx = self.m2pix(self.inflate + self.object_type.bbox.dimensions[0])
            self.ly = self.m2pix(self.inflate + self.object_type.bbox.dimensions[1])

        else:

            # let's assume that x/y dimensions are the same
            self.lx = self.m2pix(self.inflate + self.object_type.bbox.dimensions[1])
            self.ly = self.m2pix(self.inflate + self.object_type.bbox.dimensions[2])

        # if (45 < abs(self.rpy[0]) < 135 and -45 < self.rpy[0] < 45) or (-45 < abs(self.rpy[0]) < 45 and -45 < abs(self.rpy[1]) < 45):

            # self.setRotation(-self.rpy[2])

        # else:

        # https://stackoverflow.com/questions/5782658/extracting-yaw-from-a-quaternion

        q = tf.transformations.quaternion_from_euler(*self.rpy)

        q[0] = 0.0
        q[1] = 0.0

        mag = math.sqrt(q[2] * q[2] + q[3] * q[3])
        q[2] /= mag
        q[3] /= mag

        # rpy = tf.transformations.euler_from_quaternion(q)
        # ang = rpy[2]

        ang = 2 * math.acos(q[3])

        print ang / (2 * 3.14) * 360

        self.update()

    def update_text(self):

        if self.desc is None:
            return

        desc = ""
        desc += translate("ObjectItem", "ID: ") + self.object_id

        if self.hover:

            desc += "\n" + translate("ObjectItem",
                                     "TYPE: ") + self.object_type.name
            desc += "\n" + self.get_pos_str()

        self.desc.set_content(desc)

    def hover_changed(self):

        self.update_text()
        self.update()

    def boundingRect(self):

        if not self.scene():

            return QtCore.QRectF()

        lx = self.hover_ratio * self.lx
        ly = self.hover_ratio * self.ly
        p = 1.0
        return QtCore.QRectF(-lx / 2 - p, -ly / 2 - p, lx + 2 * p, ly + 2 * p)

    def paint(self, painter, option, widget):

        if not self.scene():
            return

        painter.setClipRect(option.exposedRect)
        painter.setRenderHint(QtGui.QPainter.Antialiasing)

        rr = 10

        if self.selected:

            painter.setBrush(QtCore.Qt.green)
            painter.setPen(QtCore.Qt.green)

            painter.drawRoundedRect(-self.lx / 2 * self.hover_ratio, -self.ly / 2 * self.hover_ratio,
                                    self.lx * self.hover_ratio, self.ly * self.hover_ratio, rr, rr, QtCore.Qt.RelativeSize)

        elif self.hover:

            painter.setBrush(QtCore.Qt.gray)
            painter.setPen(QtCore.Qt.gray)

            painter.drawRoundedRect(-self.lx / 2 * self.hover_ratio, -self.ly / 2 * self.hover_ratio,
                                    self.lx * self.hover_ratio, self.ly * self.hover_ratio, rr, rr, QtCore.Qt.RelativeSize)

        painter.setBrush(self.def_color)
        painter.setPen(self.def_color)

        painter.drawRoundedRect(-self.lx / 2, -self.ly / 2, self.lx,
                                self.ly, rr, rr, QtCore.Qt.RelativeSize)

        fr = 1.0 - (self.hover_ratio - 1.0)  # fill ratio

        painter.setBrush(QtCore.Qt.black)
        painter.setPen(QtCore.Qt.black)
        painter.drawRoundedRect(-self.lx / 2 * fr, -self.ly / 2 * fr,
                                self.lx * fr, self.ly * fr, rr, rr, QtCore.Qt.RelativeSize)

    def cursor_press(self):

        # TODO call base class method

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
