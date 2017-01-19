#!/usr/bin/env python

"""
Visualization of place on the table
"""

from PyQt4 import QtGui, QtCore
from object_item import ObjectItem
from posestamped_cursor_item import PoseStampedCursorItem
from touch_table_item import TouchTableItem
from point_item import PointItem

translate = QtCore.QCoreApplication.translate


class PlaceItem(ObjectItem):

    """The class to visualize place pose for a given object / object type.

    """

    def __init__(self, scene, rpm,  caption,  x, y, object_type,  object_id=None,  yaw=0,  place_pose_changed=None,  selected=False, fixed=False):

        self.in_collision = False
        self.caption = caption

        super(PlaceItem, self).__init__(scene, rpm, object_id, object_type,  x, y,  yaw)

        self.update_text()
        self.fixed = fixed
        self.place_pose_changed = place_pose_changed
        if not self.fixed:
            self.set_color(QtCore.Qt.white)
            self.point = PointItem(scene, rpm, 0, 0, self,  self.point_changed)  # TODO option to pass pixels?
            self.point.setPos(self.boundingRect().topLeft())

    def update_text(self):

        if self.desc is None:
            return

        desc = []
        desc.append(self.caption)

        if self.object_id is not None:
            desc.append(translate("ObjectItem", "ID: ") + str(self.object_id))
        else:
            desc.append(translate("ObjectItem", "TYPE: ") + self.object_type.name)

        if self.hover:

            desc.append(self.get_pos_str())

        self.desc.set_content(desc)

    def cursor_release(self):

        # TODO call base class method

        if self.place_pose_changed is not None:
            self.place_pose_changed(self.get_pos(),  self.rotation())

    def cursor_press(self):

        pass

    def point_changed(self, pt,  finished=False):

        from math import atan2, pi

        # follow angle between "free" point and object center, after release put object back on topLeft corner
        angle = atan2(self.point.scenePos().y()-self.scenePos().y(),  self.point.scenePos().x()-self.scenePos().x())/(2*pi)*360+135
        self.setRotation(angle)
        self.point.setRotation(-angle)

        self._update_desc_pos()

        if finished:

            self.point.setPos(self.boundingRect().topLeft())

            if self.place_pose_changed is not None:
                self.place_pose_changed(self.get_pos(),  self.rotation())

    def item_moved(self):

        for it in self.collidingItems():

            if isinstance(it, PoseStampedCursorItem) or isinstance(it,  TouchTableItem):
                continue

            if isinstance(it, ObjectItem):
                self.in_collision = True
                self.set_color(QtCore.Qt.red)
                break
        else:
            self.in_collision = False
            self.set_color(QtCore.Qt.white)
