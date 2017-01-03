#!/usr/bin/env python

from PyQt4 import QtGui, QtCore
from item import Item
import rospy
from geometry_msgs.msg import PoseStamped
from touch_table_item import TouchTableItem
from desc_item import DescItem

# TODO optional filtering (kalman?)
# TODO option to select when hovered for some time (instead of 'click')


class PoseStampedCursorItemHelper(QtCore.QObject):

    def __init__(self, topic, world_frame, cb):

        super(PoseStampedCursorItemHelper, self).__init__()

        self.ps_sub = rospy.Subscriber(topic, PoseStamped, self.ps_cb)
        self.cb = cb

        QtCore.QObject.connect(self, QtCore.SIGNAL('ps_cb'), self.ps_cb_evt)

    def ps_cb(self, msg):
        # TODO check frame / transform
        # if self.msg.header.frame_id != self.world_frame:
        #    return

        self.emit(QtCore.SIGNAL('ps_cb'), msg)

    def ps_cb_evt(self, msg):

        self.cb(msg)


class PoseStampedCursorItem(Item):

    def __init__(self, scene, rpm, topic, offset=(0.0, 0.0), world_frame="marker"):

        super(PoseStampedCursorItem, self).__init__(scene, rpm, 0.5, 0.5)

        self.offset = offset
        self.click = False

        self.pointed_item = None
        self.pointed_time = None
        self.last_move = None
        self.pointed_item_clicked = False

        self.fx = 0.0
        self.fy = 0.0

        self.helper = PoseStampedCursorItemHelper(topic, world_frame, self.cb)

        self.setZValue(200)

        self.last_pt = None

        self.setFlag(QtGui.QGraphicsItem.ItemIsMovable, True)
        self.setFlag(QtGui.QGraphicsItem.ItemIsSelectable, True)
        self.setCacheMode(QtGui.QGraphicsItem.DeviceCoordinateCache)

        self.mouse = False

    # for debugging purposes
    def mouseDoubleClickEvent(self, evt):

        if self.click:
            self.handle_pt(self.pos(), click=False)
        else:
            self.handle_pt(self.pos(), click=True)

    def mouseMoveEvent(self, event):

        self.mouse = True
        self.handle_pt(self.pos())

        super(Item, self).mouseMoveEvent(event)

    def boundingRect(self):

        return QtCore.QRectF(-20, -20, 40, 40)

    def paint(self, painter, option, widget):

        painter.setClipRect(option.exposedRect)
        painter.setRenderHint(QtGui.QPainter.Antialiasing)

        if self.pointed_item_clicked:
            painter.setBrush(QtCore.Qt.red)
        else:
            painter.setBrush(QtCore.Qt.green)

        painter.setOpacity(0.5)
        painter.drawEllipse(QtCore.QPoint(0, 0), 20, 20)

    def cb(self, msg):

        if not self.isEnabled():
            return

        x = self.m2pix(msg.pose.position.x + self.offset[0])
        y = self.m2pix(msg.pose.position.y + self.offset[1])

        self.fx = 0.7 * self.fx + 0.3 * x
        self.fy = 0.7 * self.fy + 0.3 * y

        pt = QtCore.QPoint(self.fx, self.fy)
        # TODO omezit pohyb kurzoru na rozmery sceny (scene_origin, scene_size)

        self.handle_pt(pt, False)

    def handle_pt(self, pt, click=None):

        self.setPos(pt)

        if self.last_pt is None:
            self.last_pt = pt
            return

        if self.pointed_item is not None and not self.collidesWithItem(self.pointed_item):
            self.pointed_item.set_hover(False, self)
            self.pointed_item = None

        if self.pointed_item is None:

            for it in self.scene().items():

                if isinstance(it, PoseStampedCursorItem) or isinstance(it, TouchTableItem) or isinstance(it, DescItem):
                    continue  # TODO make some common class for cursors

                if self.collidesWithItem(it):

                    if (rospy.Time.now() - it.last_pointed) < rospy.Duration(3.0):
                        continue

                    rospy.logdebug("new pointed item: " + it.__class__.__name__)
                    it.set_hover(True, self)
                    self.pointed_item = it
                    self.pointed_time = rospy.Time.now()
                    self.last_move = self.pointed_time
                    self.pointed_item_clicked = False
                    break

        if self.pointed_item is not None:

            if (rospy.Time.now() - self.pointed_time) > rospy.Duration(2.0):

                if not self.pointed_item_clicked:

                    self.pointed_item.cursor_press()
                    self.pointed_item_clicked = True

                    if self.pointed_item.fixed:

                        self.pointed_item.cursor_release()
                        self.pointed_item.set_hover(False, self)
                        self.pointed_item.last_pointed = rospy.Time.now()
                        self.pointed_item = None
                        self.pointed_item_clicked = False
                        self.last_pt = pt
                        self.update()
                        return

                self.pointed_item.moveBy(pt.x() - self.last_pt.x(), pt.y() - self.last_pt.y())
                self.pointed_item.item_moved()

                mm = max(abs(pt.x() - self.last_pt.x()), abs(pt.y() - self.last_pt.y()))

                if mm > 5:

                    self.last_move = rospy.Time.now()

                if (rospy.Time.now() - self.last_move) > rospy.Duration(2.0) and (self.last_move - self.pointed_time) > rospy.Duration(3.0):

                    rospy.logdebug("releasing pointed item: " + self.pointed_item.__class__.__name__)
                    self.pointed_item.set_hover(False, self)
                    self.pointed_item.cursor_release()
                    self.pointed_item.last_pointed = rospy.Time.now()
                    self.pointed_item = None
                    self.pointed_item_clicked = False

            else:

                mm = max(abs(pt.x() - self.last_pt.x()), abs(pt.y() - self.last_pt.y()))

                if mm > 5:

                    self.pointed_item.set_hover(False, self)
                    self.pointed_item = None

        self.last_pt = pt
        self.update()
