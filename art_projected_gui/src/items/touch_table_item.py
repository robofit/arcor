#!/usr/bin/env python

from PyQt4 import QtGui, QtCore
from item import Item
import rospy
from art_msgs.msg import Touch


class TouchPointItem(Item):

    def __init__(self,  scene,  rpm,  x,  y,  parent):

        self.pointed_item = None
        self.offset = (0, 0)

        super(TouchPointItem, self).__init__(scene, rpm, x, y,  parent)

        self.set_poss(x,  y)

    def boundingRect(self):

        return QtCore.QRectF(-20, -20, 40, 40)

    def end_of_touch(self):

        if self.pointed_item is not None:

            self.pointed_item.set_hover(False, self)
            self.cursor_release()
            self.pointed_item = None

    def set_poss(self,  x,  y):

        print "x: " + str(x) + ", y: " + str(y)
        # super(TouchPointItem, self).set_pos(x,  y)
        self.setPos(self.m2pix(x), self.m2pix(y))
        self.update()

        if self.pointed_item is None:

            for it in self.scene().items():

                # TODO skip certain types of items?

                if self.collidesWithItem(it):

                    it.set_hover(True, self)
                    self.pointed_item = it
                    self.pointed_item.cursor_press()
                    self.pointed_item.cursor_release()

                    my_pos = self.get_pos()
                    it_pos = self.pointed_item.get_pos()

                    self.offset = (my_pos[0]-it_pos[0],  my_pos[1]-it_pos[1])

        else:

            self.pointed_item.set_pos(x+self.offset[0], y+self.offset[1])
            self.pointed_item.item_moved()

    def paint(self, painter, option, widget):

        painter.setClipRect(option.exposedRect)
        painter.setRenderHint(QtGui.QPainter.Antialiasing)

        if self.pointed_item is not None:
            painter.setBrush(QtCore.Qt.red)
        else:
            painter.setBrush(QtCore.Qt.green)

        painter.setOpacity(0.5)
        painter.drawEllipse(QtCore.QPoint(0, 0), 20, 20)


# TODO make a common helper class from this (also used in PoseStampedCursorItem)
class TouchTableItemHelper(QtCore.QObject):

    def __init__(self, topic, world_frame, cb):

        super(TouchTableItemHelper, self).__init__()

        self.ps_sub = rospy.Subscriber(topic, Touch, self.ps_cb)
        self.cb = cb

        QtCore.QObject.connect(self, QtCore.SIGNAL('ps_cb'), self.ps_cb_evt)

    def ps_cb(self, msg):

        self.emit(QtCore.SIGNAL('ps_cb'), msg)

    def ps_cb_evt(self, msg):

        self.cb(msg)


class TouchTableItem(Item):

    # TODO display corners of touchable area
    def __init__(self, scene, rpm, topic, items_to_disable_on_touch=[],  world_frame="marker"):

        super(TouchTableItem, self).__init__(scene, rpm, 0.0, 0.0)

        self.scene = scene
        self.rpm = rpm
        self.touch_points = {}

        self.items_to_disable_on_touch = items_to_disable_on_touch

        self.helper = TouchTableItemHelper(topic, world_frame, self.touch_cb)

        self.setZValue(200)

    def boundingRect(self):

        return QtCore.QRectF(0, 0, self.scene.width(), self.scene.height())  # TODO is this ok?

    def paint(self, painter, option, widget):

        pass

    def touch_cb(self,  msg):

        # TODO delete all points on disable?
        if not self.isEnabled():
            return

        touches = len(self.touch_points)

        if msg.id not in self.touch_points:

            # TODO check frame_id
            self.touch_points[msg.id] = TouchPointItem(self.scene,  self.rpm,  msg.point.point.x,  msg.point.point.y,  self)

        else:

            if not msg.touch:

                self.touch_points[msg.id].end_of_touch()
                self.scene.removeItem(self.touch_points[msg.id])
                del self.touch_points[msg.id]

            else:

                self.touch_points[msg.id].set_poss(msg.point.point.x,  msg.point.point.y)

        # print "touches: " + str(touches)
        # print "len(self.touch_points): " + str(len(self.touch_points))

        if len(self.touch_points) > 0 and touches == 0:

            # print "disabling"

            for cur in self.items_to_disable_on_touch:
                cur.set_enabled(False,  True)
                # print "it"

        if len(self.touch_points) == 0 and touches > 0:

            # print "enabling"

            for cur in self.items_to_disable_on_touch:
                cur.set_enabled(True,  True)
                # print "it"
