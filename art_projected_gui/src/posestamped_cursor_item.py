#!/usr/bin/env python

from PyQt4 import QtGui, QtCore
from item import Item
import rospy
from geometry_msgs.msg import PoseStamped

# TODO optional filtering (kalman?)
# TODO option to select when hovered for some time (instead of 'click')

class PoseStampedCursorItemHelper(QtCore.QObject):

    def __init__(self,  topic,  world_frame,  cb):

        super(PoseStampedCursorItemHelper,  self).__init__()

        self.ps_sub = rospy.Subscriber(topic,  PoseStamped,  self.ps_cb)
        self.cb = cb

        QtCore.QObject.connect(self, QtCore.SIGNAL('ps_cb'), self.ps_cb_evt)

    def ps_cb(self,  msg):

        # TODO check frame / transform
        #if self.msg.header.frame_id != self.world_frame:
        #    return

        self.emit(QtCore.SIGNAL('ps_cb'),  msg)

    def ps_cb_evt(self,  msg):

        self.cb(msg)

class PoseStampedCursorItem(Item):

    def __init__(self,  scene,  rpm,   topic,  offset = (0.1,  0.1),  world_frame="marker"):

        super(PoseStampedCursorItem,  self).__init__(scene,  rpm,  0.2,  0.2)

        self.offset = offset
        self.click = False

        self.catched_item = None

        self.helper = PoseStampedCursorItemHelper(topic,  world_frame,  self.cb)

        self.setZValue(200)

        self.last_pt = None

        self.setFlag(QtGui.QGraphicsItem.ItemIsMovable, True)
        self.setFlag(QtGui.QGraphicsItem.ItemIsSelectable, True)

        self.mouse = False

    # for debugging purposes
    def mouseDoubleClickEvent(self,  evt):

        if self.click: self.handle_pt(self.pos(),  click = False)
        else: self.handle_pt(self.pos(),  click = True)

    def mouseMoveEvent(self, event):

        self.mouse = True
        self.handle_pt(self.pos())

        super(Item, self).mouseMoveEvent(event)

    def boundingRect(self):

        return QtCore.QRectF(-10, -10, 20, 20)

    def paint(self, painter, option, widget):

        if self.click: painter.setBrush(QtCore.Qt.red)
        else: painter.setBrush(QtCore.Qt.green)
        painter.setOpacity(0.5)
        painter.drawEllipse(QtCore.QPoint(0,  0), 20,  20)

    def cb(self,  msg):

        x = self.m2pix(msg.pose.position.x + self.offset[0])
        y = self.m2pix(msg.pose.position.y + self.offset[1])
        pt = QtCore.QPoint(x,  y)

        th = 0.02

        click = None

        # TODO smarter click (difference from long-term average?)
        if msg.pose.position.z < 0.2 - th:
            click = True
        elif msg.pose.position.z > 0.2 + th and self.click:
            click = False

        self.mouse = False
        self.handle_pt(pt,  click)

    def handle_pt(self,  pt,  click = None):

        if self.last_pt is None:
            self.last_pt = pt
            return

        if len(self.collidingItems())==0: click=False

        for it in self.scene().items():

            if isinstance(it,  PoseStampedCursorItem): continue # TODO make some common class for cursors

            # TODO take z-value into account?

            if self.collidesWithItem(it):

                # TODO ted se nastavuje hover jen pro prvni polozku - chtelo by to podle prekryvu s kurzorem ;)
                if self.catched_item is None: it.set_hover(True,  self)

                if click is not None:
                    if click and not self.click: it.cursor_press()
                    if self.click and not click:
                        self.catched_item = None
                        it.cursor_release()
                        break

                if self.click and not it.fixed:

                        if self.catched_item is None: self.catched_item = it
                        break

                break

            else:

                it.set_hover(False,  self)

        if self.catched_item is not None:

            self.catched_item.moveBy(pt.x()-self.last_pt.x(),  pt.y()-self.last_pt.y())
            self.catched_item.item_moved()

        if not self.mouse: self.setPos(pt)

        self.last_pt = pt
        if click is not None: self.click = click
        self.update()

