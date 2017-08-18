#!/usr/bin/env python

from PyQt4 import QtGui, QtCore
import rospy
from art_projected_gui.helpers import conversions

# spolecny predek vseho ve scene


class Item(QtGui.QGraphicsItem):

    X = 0
    Y = 1
    Z = 2

    def __init__(self, scene, x, y, z=0, parent=None):

        super(Item, self).__init__(parent=parent, scene=scene)

        self.hover = False
        self.hover_sources = []
        self.fixed = True
        self.cursor_press_at = rospy.Time(0)
        self.default_font = 'Arial'
        self.last_pointed = rospy.Time(0)
        self.position = [x, y, z]  # position in meters

        self.setVisible(True)
        # self.setAcceptHoverEvents(True)
        self.setEnabled(True)
        self.setActive(True)
        self.setCacheMode(QtGui.QGraphicsItem.ItemCoordinateCache)

        self.set_pos(x, y, z)

    def get_font_size(self, f=1.0):

        return 8 / 1280.0 * self.scene().rpm * f

    def m2pix(self, x, y=None):

        if y is None:
            return x * self.scene().rpm

        return (self.m2pix(x), self.scene().height() - self.m2pix(y))

    def pix2m(self, x, y=None):

        if y is None:
            return x / self.scene().rpm

        return (self.pix2m(x), self.pix2m(self.scene().height() - y))

    # world coordinates to scene coords - y has to be inverted
    def set_pos(self, x, y, z=None, parent_coords=False, yaw=None):

        self.position[0] = x
        self.position[1] = y
        if z is not None:
            self.position[2] = z

        (px, py) = self.m2pix(x, y)

        # we usually want to work with scene/world coordinates
        if self.parentItem() and not parent_coords:

            self.setPos(self.parentItem().mapFromScene(px, py))

        else:

            self.setPos(px, py)

        if yaw is not None:
            self.setRotation(-yaw)

    def get_pos(self, pixels=False):

        # TODO return self.pos ?

        pos = self.scenePos()

        if not pixels:
            return self.pix2m(pos.x(), pos.y())
        else:
            return (pos.x(), pos.y())

    def get_pos_str(self):

        return conversions.pos2str(self.position)

    def _width(self):

        return self.boundingRect().width()

    def _height(self):

        return self.boundingRect().height()

    def _place_childs_horizontally(self, y, padding, items):

        # TODO test if items are childs of this item?

        if len(items) == 1:  # if there is one item - center it

            items[0].setPos((self._width() - items[0]._width()) / 2, y)

        elif len(items) > 1:  # more than one - place them with equal space between them (with padding on left and right)

            total_width = 0

            for it in items:
                total_width += it._width()

            inner_space = (self._width() - 2 * padding -
                           total_width) / len(items) - 1

            items[0].setPos(padding, y)

            for idx in range(1, len(items)):

                items[idx].setPos(items[idx - 1].x() +
                                  items[idx - 1]._width() + inner_space, y)

    def set_enabled(self, state, also_set_visibility=False):

        self.setEnabled(state)
        if also_set_visibility:
            self.setVisible(state)

    def hover_changed(self):

        self.update()

    def set_hover(self, state, source):

        if state:
            if source not in self.hover_sources:
                self.hover_sources.append(source)
        else:
            if source in self.hover_sources:
                self.hover_sources.remove(source)

        if len(self.hover_sources) == 0:
            if self.hover:
                self.hover = False
                self.hover_changed()

        else:
            if not self.hover:
                self.hover = True
                self.hover_changed()

    def cursor_press(self):

        self.cursor_press_at = rospy.Time.now()

    def cursor_release(self):

        if rospy.Time.now() - self.cursor_press_at < rospy.Duration(1.0):
            self.cursor_click()

    def cursor_click(self):

        pass

    def item_moved(self):

        pass

    def mouseDoubleClickEvent(self, evt):

        self.cursor_click()

    def boundingRect(self):

        raise NotImplementedError("Please implement this method")

    def paint(self, painter, option, widget):

        raise NotImplementedError("Please implement this method")
