#!/usr/bin/env python

"""
Visualization of detected object(s).

"""

from PyQt4 import QtGui, QtCore
from item import Item
from desc_item import DescItem

translate = QtCore.QCoreApplication.translate


class ObjectItem(Item):

    """The class to visualize (detected) object.

    It currently supports only rotation around z-axis.

    """

    def __init__(self, scene, object_id, object_type, x, y, yaw,  sel_cb=None, selected=False):

        self.object_id = object_id
        self.selected = selected
        self.sel_cb = sel_cb
        self.object_type = object_type  # TODO check bbox type and use rectangle (used now) / ellipse, consider other angles
        self.inflate = 1.0
        self.hover_ratio = 1.1
        self.def_color = QtCore.Qt.gray

        self.desc = None

        super(ObjectItem, self).__init__(scene, x, y)

        self.desc = DescItem(scene, 0,  0, self)
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

            # make upper left corner of description aligned with left extent of the (possibly rotated) object bounding box (highlight area)
            self.desc.setPos(self.mapFromScene(self.x()-self.sceneBoundingRect().width()/2,  self.y()+self.sceneBoundingRect().height()/2 + self.m2pix(0.01)))

    def set_pos(self, x, y, parent_coords=False,  yaw=None):

        super(ObjectItem, self).set_pos(x, y,  parent_coords,  yaw)
        self._update_desc_pos()
        self.update_text()

    def update_text(self):

        if self.desc is None:
            return

        desc = ""
        desc += translate("ObjectItem", "ID: ") + self.object_id

        if self.hover:

            desc += "\n" + translate("ObjectItem", "TYPE: ") + self.object_type.name
            desc += "\n" + self.get_pos_str()

        self.desc.set_content(desc)

    def hover_changed(self):

        self.update_text()
        self.update()

    def boundingRect(self):

        if not self.scene():

            return QtCore.QRectF()

        lx = self.hover_ratio*self.inflate*self.m2pix(self.object_type.bbox.dimensions[0])
        ly = self.hover_ratio*self.inflate*self.m2pix(self.object_type.bbox.dimensions[1])
        p = 1.0
        return QtCore.QRectF(-lx / 2 - p, -ly / 2 - p, lx + 2 * p, ly + 2 * p)

    def paint(self, painter, option, widget):

        if not self.scene():
            return

        painter.setClipRect(option.exposedRect)
        painter.setRenderHint(QtGui.QPainter.Antialiasing)

        lx = self.inflate*self.m2pix(self.object_type.bbox.dimensions[0])
        ly = self.inflate*self.m2pix(self.object_type.bbox.dimensions[1])

        rr = 10

        if self.selected:

            painter.setBrush(QtCore.Qt.green)
            painter.setPen(QtCore.Qt.green)

            painter.drawRoundedRect(-lx/2*self.hover_ratio,  -ly/2*self.hover_ratio,  lx*self.hover_ratio,  ly*self.hover_ratio,  rr, rr,  QtCore.Qt.RelativeSize)

        elif self.hover:

            painter.setBrush(QtCore.Qt.gray)
            painter.setPen(QtCore.Qt.gray)

            painter.drawRoundedRect(-lx/2*self.hover_ratio,  -ly/2*self.hover_ratio,  lx*self.hover_ratio,  ly*self.hover_ratio,  rr, rr,  QtCore.Qt.RelativeSize)

        painter.setBrush(self.def_color)
        painter.setPen(self.def_color)

        painter.drawRoundedRect(-lx/2,  -ly/2,  lx,  ly,  rr, rr,  QtCore.Qt.RelativeSize)

        fr = 1.0 - (self.hover_ratio - 1.0)  # fill ratio

        painter.setBrush(QtCore.Qt.black)
        painter.setPen(QtCore.Qt.black)
        painter.drawRoundedRect(-lx/2*fr,  -ly/2*fr,  lx*fr,  ly*fr,  rr, rr,  QtCore.Qt.RelativeSize)

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
