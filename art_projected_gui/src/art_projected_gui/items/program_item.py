#!/usr/bin/env python

from PyQt4 import QtGui, QtCore
from item import Item
from art_msgs.msg import ProgramItem as ProgIt
from geometry_msgs.msg import Point32
from button_item import ButtonItem
from art_projected_gui.helpers import conversions

translate = QtCore.QCoreApplication.translate


# TODO make it based on ButtonItem
class ProgramItemItem(Item):

    def __init__(self, scene, rpm, x, y, item, parent, item_selected_cb=None):

        self.w = 180  # TODO spocitat podle rpm
        self.h = 40

        self.item_selected_cb = item_selected_cb

        super(ProgramItemItem, self).__init__(scene, rpm, x, y, parent)

        self.item = item
        self.active_item = False

        self.update_size()

    def update_size(self):

        # TODO bude se menit po kazdem nastaveni neceho...
        font = QtGui.QFont('Arial', self.get_font_size())
        metrics = QtGui.QFontMetrics(font)
        txt = self.get_text_for_item()
        self.h = metrics.height() * (txt.count('\n') + 1) + 10
        self.w = 0

        for str in txt.split('\n'):

            w = metrics.width(str) + 20 + 20  # 20 - padding, 20 - ID
            if w > self.w:
                self.w = w

        self.update()

    def cursor_click(self):

        if not self.item_req_learning():
            return
        if self.item_selected_cb is not None:
            self.item_selected_cb(self)

    def boundingRect(self):

        return QtCore.QRectF(0, 0, self.w, self.h)

    def get_text_for_item(self):

        pose_str = "x=" + str(round(self.item.place_pose.pose.position.x, 2)) + ", y=" + str(round(self.item.place_pose.pose.position.y, 2))
        obj = self.item.object

        if self.item.place_pose.pose.position.x == 0 and self.item.place_pose.pose.position.y == 0:
            pose_str = "x=??, y=??"

        if obj == "":

            obj = "??"

        if self.item.type == ProgIt.GET_READY:
            return QtCore.QCoreApplication.translate("ProgramItem", "get ready")
        elif self.item.type == ProgIt.MANIP_PICK:

            if self.item.spec == ProgIt.MANIP_ID:
                return QtCore.QCoreApplication.translate("ProgramItem", "pick object ID=")
            elif self.item.spec == ProgIt.MANIP_TYPE:
                return QtCore.QCoreApplication.translate("ProgramItem", "pick object type=")

        elif self.item.type == ProgIt.MANIP_PLACE:

            # TODO pose / polygon
            return QtCore.QCoreApplication.translate("ProgramItem", "place object at ") + pose_str

        elif self.item.type == ProgIt.MANIP_PICK_PLACE:

            if self.item.spec == ProgIt.MANIP_ID:
                return QtCore.QCoreApplication.translate("ProgramItem", "pick object") + "'" + obj + "'\n" + QtCore.QCoreApplication.translate("ProgramItem", "place to ") + pose_str
            elif self.item.spec == ProgIt.MANIP_TYPE:
                return QtCore.QCoreApplication.translate("ProgramItem", "pick object type") + "'" + obj + "'\n" + QtCore.QCoreApplication.translate("ProgramItem", "place to ") + pose_str

        elif self.item.type == ProgIt.WAIT:

            if self.item.spec == ProgIt.WAIT_FOR_USER:
                return QtCore.QCoreApplication.translate("ProgramItem", "wait for user")
            elif self.item.spec == ProgIt.WAIT_UNTIL_USER_FINISHES:
                return QtCore.QCoreApplication.translate("ProgramItem", "wait until user finishes")

    def item_req_learning(self):

        return self.item.type in [ProgIt.MANIP_PICK, ProgIt.MANIP_PLACE, ProgIt.MANIP_PICK_PLACE]

    def is_place_pose_set(self):

        return self.item.place_pose.pose.position.x != 0 and self.item.place_pose.pose.position.y != 0

    def get_place_pose(self):

        return self.item.place_pose

    def is_object_set(self):

        return self.item.object != ""

    def is_pick_polygon_set(self):

        return len(self.item.pick_polygon.polygon.points) > 0

    def get_pick_polygon_points(self):

        poly_points = []

        for pt in self.item.pick_polygon.polygon.points:

            poly_points.append((pt.x, pt.y))

        return poly_points

    def item_learned(self):

        if not self.item_req_learning():
            return True

        # TODO dalsi typy / spec
        if self.item.type == ProgIt.MANIP_PICK_PLACE:

            if self.item.object != "" and len(self.item.pick_polygon.polygon.points) > 0 and self.is_place_pose_set():
                return True

        return False

    def paint(self, painter, option, widget):

        painter.setClipRect(option.exposedRect)
        painter.setRenderHint(QtGui.QPainter.Antialiasing)

        font = QtGui.QFont('Arial', self.get_font_size(1.5))
        painter.setFont(font)
        painter.setPen(QtCore.Qt.white)

        painter.drawText(0, 20, str(self.item.id))

        rect = QtCore.QRectF(20, 0.0, self.w, self.h)

        font = QtGui.QFont('Arial', self.get_font_size())
        painter.setFont(font)

        pen = QtGui.QPen()
        if not self.hover:
            pen.setStyle(QtCore.Qt.NoPen)
        painter.setBrush(QtCore.Qt.black)
        pen.setColor(QtCore.Qt.white)

        if self.item_req_learning():

            if not self.item_learned():

                painter.setBrush(QtCore.Qt.red)

            else:

                painter.setBrush(QtCore.Qt.black)

        else:

            painter.setBrush(QtCore.Qt.darkGray)

        txt = self.get_text_for_item()

        rect.setWidth(self.w - 20)

        painter.setPen(pen)
        painter.drawRoundedRect(rect, 10.0, 10.0)
        rect.moveTo(25, 0)
        pen.setStyle(QtCore.Qt.SolidLine)
        painter.setPen(pen)
        painter.drawText(rect, QtCore.Qt.AlignLeft, txt)


class ProgramItem(Item):

    def __init__(self, scene, rpm, x, y, active_item_switched=None, program_state_changed=None):

        self.states = ['LEARNING', 'RUNNING', 'PAUSED', 'STOPPED']
        self.state = 'LEARNING'

        self.w = 180  # TODO spocitat podle rpm
        self.h = 20

        super(ProgramItem, self).__init__(scene, rpm, x, y)

        self.prog = None
        self.template = None
        self.items = []
        self.active_item = None

        self.active_item_switched = active_item_switched
        self.program_state_changed = program_state_changed
        self.fixed = False

        self.setZValue(100)

    def set_running(self):

        self.state = 'RUNNING'

    def has_prog(self):

        return self.prog is None

    def set_active(self, it=None, inst_id=None):

        if it is None and inst_id is not None:

            it = self.get_item_by_id(inst_id)

        if it is None:
            return

        if self.active_item is not None:
            self.active_item.setPos(30, self.active_item.y())
            self.active_item.active_item = False

        self.active_item = None
        self.active_item = it
        self.active_item.active_item = True
        self.active_item.update()

        self.active_item.setPos(10, self.active_item.y())

    def is_prog_learned(self):

        for it in self.items:

            if not it.item_learned():
                return False

        return True

    def get_prog(self):

        # TODO deal with more blocks
        del self.prog.blocks[0].items[:]

        for it in self.items:

            self.prog.blocks[0].items.append(it.item)

        return self.prog

    def set_prog(self, prog, template):

        self.prog = prog
        self.template = template
        self.items = []

        self.active_item = None

        cnt = 0
        max_w = 0

        # TODO Handle program groups - create ProgramGroupItem class and allow switching between groups. For now, let's assume that there will always be just one group
        for pitem in self.prog.blocks[0].items:

            if self.template:

                # to make sure that template is 'clear'
                pitem.object = ""
                del pitem.pick_polygon.polygon.points[:]
                pitem.place_pose.pose.position.x = 0
                pitem.place_pose.pose.position.y = 0
                pitem.place_pose.pose.position.z = 0

            # TODO how to place items easily? using some layout??
            self.items.append(ProgramItemItem(self.scene(), self.rpm, 0, 0, pitem, self, self.item_selected_cb))
            if len(self.items) < 2:
                self.items[-1].setPos(30, 20)
            else:
                self.items[-1].setPos(30, 10 + self.items[-2].y() + self.items[-2].h)

            if self.items[-1].w > max_w:
                max_w = self.items[-1].w
            cnt += 1

            # set first item as active
            if (self.active_item is None) and self.items[-1].item_req_learning():

                self.set_active(self.items[-1])
            # if pitem.type in self.items_req_learning:
            #    if self.active_item is None: self.active_item = pitem

        self.btn = ButtonItem(self.scene(), self.rpm, 0, 0, translate("ProgramItem", "Start"), self, self.btn_clicked)
        self.btn.setPos(10, 5 + cnt * 50)
        self.btn.set_enabled(False)

        # TODO najit max. sirku itemu a tomu prizpusobit sirku programu
        self.h = 20 + cnt * 50 + 30
        self.w = max_w + 40

        self.update()

    def btn_clicked(self, btn):

        if self.state == 'LEARNING':

            self.state = 'RUNNING'
            self.btn.set_caption(translate("ProgramItem", "Pause"))

        elif self.state == 'RUNNING':

            self.state = 'PAUSED'
            self.btn.set_caption(translate("ProgramItem", "Resume"))

        # TODO second button for STOP

        if self.program_state_changed is not None:
            self.program_state_changed(self.state)

    def item_selected_cb(self, it):

        if self.state == 'RUNNING':
            print "program running - ignoring item selection"
            return

        self.set_active(it)
        if self.active_item_switched is not None:
            self.active_item_switched()

    def update_size(self):

        self.active_item.update_size()

        max_w = 0

        for it in self.items:

            if self.active_item.w + 40 > max_w:
                max_w = self.active_item.w + 40
        self.w = max_w

        self.update()

    def program_updated(self):

        # TODO consides state
        if self.is_prog_learned():
            self.btn.set_enabled(True)

    def set_object(self, obj):

        self.active_item.item.object = obj
        self.program_updated()
        self.update_size()

    def set_place_pose(self, x, y,  yaw):

        self.active_item.item.place_pose.pose.position.x = x
        self.active_item.item.place_pose.pose.position.y = y
        self.active_item.item.place_pose.pose.orientation = conversions.yaw2quaternion(yaw)

        self.program_updated()
        self.update_size()

    def set_polygon(self, pts):

        del self.active_item.item.pick_polygon.polygon.points[:]

        for pt in pts:

            self.active_item.item.pick_polygon.polygon.points.append(Point32(pt[0], pt[1], 0))

        self.program_updated()
        self.update_size()

    def boundingRect(self):

        return QtCore.QRectF(0, -30, self.w, self.h + 30)

    def get_item_by_id(self, id):

        for it in self.items:

            if it.item.id == id:

                return it

        return None

    def paint(self, painter, option, widget):

        if self.prog is None:
            return

        painter.setClipRect(option.exposedRect)
        painter.setRenderHint(QtGui.QPainter.Antialiasing)

        font = QtGui.QFont('Arial', 14)
        painter.setFont(font)
        painter.setPen(QtCore.Qt.white)

        painter.drawText(0, -10, translate("ProgramItem", "Program") + " ID=" + str(self.prog.header.id))

        pen = QtGui.QPen()
        pen.setStyle(QtCore.Qt.NoPen)
        painter.setPen(pen)

        painter.setBrush(QtCore.Qt.gray)
        painter.setOpacity(0.5)
        # TODO projit itemy a zjistit skutecnou velikost (muze byt ruzna)
        painter.drawRect(0, 0, self.w, self.h)
