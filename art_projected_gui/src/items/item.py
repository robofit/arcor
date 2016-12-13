#!/usr/bin/env python

from PyQt4 import QtGui, QtCore
import rospy


# spolecny predek vseho ve scene
class Item(QtGui.QGraphicsItem):

    def __init__(self, scene, rpm, x, y, parent=None):

        super(Item, self).__init__(parent=parent, scene=scene)

        self.rpm = rpm
        self.set_pos(x, y, parent is not None)
        self.hover = False
        self.hover_sources = []
        self.fixed = True
        self.cursor_press_at = rospy.Time(0)
        self.default_font = 'Arial'
        self.last_pointed = rospy.Time(0)

        self.setVisible(True)
        # self.setAcceptHoverEvents(True)
        self.setEnabled(True)
        self.setActive(True)
        self.setCacheMode(QtGui.QGraphicsItem.ItemCoordinateCache)

    def enable(self):
        self.setVisible(True)
        self.setEnabled(True)

    def disable(self):

        self.setVisible(False)
        self.setEnabled(False)

    def get_font_size(self, f=1.0):

        return 12 / 1280.0 * self.rpm * f

    def m2pix(self, m):

        return m * self.rpm

    def pix2m(self, p):

        return p / self.rpm

    # world coordinates to scene coords
    def set_pos(self, x, y, parent_coords=False):

        # we usually want to work with scene/world coordinates
        if self.parentItem() and not parent_coords:

            ppos = self.parentItem().scenePos()

            self.setPos(self.m2pix(x) - ppos.x(), self.m2pix(y) - ppos.y())

        else:

            self.setPos(self.m2pix(x), self.m2pix(y))

    def get_pos(self, pixels=False):

        pos = self.scenePos()

        if not pixels:
            return (self.pix2m(pos.x()), self.pix2m(pos.y()))
        else:
            return (pos.x(), pos.y())

    def get_pos_str(self):

        (x, y) = self.get_pos()
        # TODO fixed width format
        return "[X: " + str(round(x, 3)).ljust(5, '0') + ", Y: " + str(round(y, 3)).ljust(5, '0') + "]"

    def set_enabled(self, state):

        self.setEnabled(state)
        self.update()

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

    def boundingRect(self):

        raise NotImplementedError("Please implement this method")

    def paint(self, painter, option, widget):

        raise NotImplementedError("Please implement this method")
