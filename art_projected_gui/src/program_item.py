#!/usr/bin/env python

from PyQt4 import QtGui, QtCore
from item import Item
from art_msgs.msg import ProgramItem as ProgIt
from geometry_msgs.msg import Point32

class ProgramItem(Item):

    def __init__(self,  scene,  rpm,  x,  y,  active_item_switched=None):

        self.w = 180 # TODO spocitat podle rpm
        self.h = 20

        super(ProgramItem,  self).__init__(scene,  rpm,  x,  y)

        self.prog = None
        self.template = None

        self.items_req_learning = [ProgIt.MANIP_PICK,  ProgIt.MANIP_PLACE,  ProgIt.MANIP_PICK_PLACE]

        self.items_to_be_learned = []

        self.active_item_switched = active_item_switched

        self.setFlag(QtGui.QGraphicsItem.ItemIsMovable, True)
        self.setFlag(QtGui.QGraphicsItem.ItemIsSelectable, True)

    def has_prog(self):

        return self.prog is None

    def set_prog(self,  prog,  template):

        self.prog = prog
        self.template = template

        self.h = len(prog.items)*60

        self.items_to_be_learned = []

        self.active_item = None

        for pitem in self.prog.items:

            if pitem.type in self.items_req_learning:
                if self.active_item is None: self.active_item = pitem
                self.items_to_be_learned.append(pitem.id)

        self.update()

    def set_object(self,  obj):

        self.active_item.object = obj
        self.update()

    def set_place_pose(self,  x,  y):

        self.active_item.place_pose.pose.position.x = x
        self.active_item.place_pose.pose.position.y = y

    def set_polygon(self,  pts):

        del self.active_item.pick_polygon.polygon.points[:]

        for pt in pts:

            self.active_item.pick_polygon.polygon.points.append(Point32(pt[0],  pt[1],  0))

    def mouseDoubleClickEvent(self,  evt):

        # TODO prepnout na poklikanou polozku, zatim jen na nasledujici/prvni

        if self.active_item.id == self.items_to_be_learned[-1]:

            self.active_item = self.get_item_by_id(self.items_to_be_learned[0])

        else:

            next_idx = self.items_to_be_learned.index(self.active_item.id)+1
            self.active_item = self.get_item_by_id(self.items_to_be_learned[next_idx])

        if self.active_item_switched is not None: self.active_item_switched()

        self.update()


    def boundingRect(self):

        #w = self.m2pix(self.w)
        #h = self.m2pix(self.h)

        return QtCore.QRectF(0,  0, self.w, self.h)

    def item_learned(self,  it):

        #if it.type not in self.items_req_learning: return True

        # TODO dalsi typy / spec
        if it.type== ProgIt.MANIP_PICK_PLACE:

            if it.object != "" and len(it.pick_polygon.polygon.points) > 0 and it.place_pose.pose.position.x != 0 and it.place_pose.pose.position.y != 0: return True

        return False

    def get_text_for_item(self,  it):

        pose_str = "x=" + str(round(it.place_pose.pose.position.x,  2)) + ", y=" + str(round(it.place_pose.pose.position.y,  2))
        obj = it.object

        if it.place_pose.pose.position.x == 0 and it.place_pose.pose.position.y == 0:
             pose_str = "x=??, y=??"

        if obj == "":

            obj = "??"

        if it.type == ProgIt.GET_READY:
            return "[" + str(it.id) + "] " + QtCore.QCoreApplication.translate("ProgramItem", "get ready")
        elif it.type == ProgIt.MANIP_PICK:

            if it.spec == ProgIt.MANIP_ID:
                return "[" + str(it.id) + "] " + QtCore.QCoreApplication.translate("ProgramItem", "pick object ID=")
            elif it.spec == ProgIt.MANIP_TYPE:
                return "[" + str(it.id) + "] " +QtCore.QCoreApplication.translate("ProgramItem", "pick object type=")

        elif it.type == ProgIt.MANIP_PLACE:

            # TODO pose / polygon
            return "[" + str(it.id) + "] " + QtCore.QCoreApplication.translate("ProgramItem", "place object at ")  + pose_str

        elif it.type == ProgIt.MANIP_PICK_PLACE:

            if it.spec == ProgIt.MANIP_ID:
                return "[" + str(it.id) + "] " + QtCore.QCoreApplication.translate("ProgramItem", "pick object ID=") + obj + QtCore.QCoreApplication.translate("ProgramItem", ", place to ") + pose_str
            elif it.spec == ProgIt.MANIP_TYPE:
                return "[" + str(it.id) + "] " +QtCore.QCoreApplication.translate("ProgramItem", "pick object type=") + obj + QtCore.QCoreApplication.translate("ProgramItem", ", place to ") + pose_str

        elif it.type == ProgIt.WAIT:

            if it.spec == ProgIt.WAIT_FOR_USER:
                return "[" + str(it.id) + "] " + QtCore.QCoreApplication.translate("ProgramItem", "wait for user")
            elif it.spec == ProgIt.WAIT_UNTIL_USER_FINISHES:
                return "[" + str(it.id) + "] " + QtCore.QCoreApplication.translate("ProgramItem", "wait until user finishes")

    def get_item_by_id(self,  id):

        for it in self.prog.items:

            if it.id == id:

                return it

        return None

    def paint(self, painter, option, widget):

        if self.prog is None: return

        rect = QtCore.QRectF(5, 0.0, self.w, 40.0)

        font = QtGui.QFont('Arial', 12)
        painter.setFont(font);
        painter.setPen(QtCore.Qt.white)
        metrics = QtGui.QFontMetrics(font)

        # TODO zvyraznovat i naucenou polozku
        for pitem in self.prog.items:

            if pitem.id in self.items_to_be_learned  and not self.item_learned(pitem):

                if pitem.id == self.active_item.id:

                    painter.setBrush(QtCore.Qt.red)
                    painter.setPen(QtCore.Qt.white)

                else:

                    painter.setBrush(QtCore.Qt.black)
                    painter.setPen(QtCore.Qt.red)

            else:

                painter.setBrush(QtCore.Qt.black)
                painter.setPen(QtCore.Qt.white)

            txt = self.get_text_for_item(pitem)
            rect.setWidth(metrics.width(txt)+20)

            painter.drawRoundedRect(rect, 10.0, 10.0);
            painter.drawText(rect,  QtCore.Qt.AlignLeft,  txt)

            rect.moveTo(5, rect.y()+60)


