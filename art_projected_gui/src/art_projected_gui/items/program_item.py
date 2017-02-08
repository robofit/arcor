#!/usr/bin/env python

from PyQt4 import QtGui, QtCore
from item import Item
from art_msgs.msg import ProgramItem as ProgIt
from geometry_msgs.msg import Point32
from button_item import ButtonItem
from art_projected_gui.helpers import conversions
from list_item import ListItem

translate = QtCore.QCoreApplication.translate


class ProgramItem(Item):

    def __init__(self, scene, rpm, x, y, program_helper, done_cb=None, item_switched_cb=None):

        self.w = 100
        self.h = 100

        self.done_cb = done_cb
        self.item_switched_cb = item_switched_cb

        super(ProgramItem, self).__init__(scene, rpm, x, y)

        self.w = self.m2pix(0.2)
        self.h = self.m2pix(0.25)
        self.sp = self.m2pix(0.01)

        self.ph = program_helper

        # block "view"
        self.block_finished_btn = ButtonItem(self.scene(), self.rpm, 0, 0, translate("ProgramItem", "Done"), self, self.block_finished_btn_cb)
        self.block_edit_btn = ButtonItem(self.scene(), self.rpm, 0, 0, translate("ProgramItem", "Edit"), self, self.block_edit_btn_cb)
        self.block_on_failure_btn = ButtonItem(self.scene(), self.rpm, 0, 0, translate("ProgramItem", "On fail"), self, self.block_on_failure_btn)

        bdata = []

        self.blocks_map = {} # map from indexes (key) to block_id (value)

        block_id = self.ph.get_first_block_id()

        while block_id != 0:

            bmsg = self.ph.get_block_msg(block_id)

            bdata.append("Block ID: " + str(block_id) + "\nName: " + bmsg.name)
            self.blocks_map[len(bdata)-1] = block_id
            # TODO disp. more
            # TODO zobrazit upozorneni kdyz ma blok jine on_failure nez konec programu

            block_id = self.ph.get_block_on_success(block_id)

            # program je zacykleny
            if block_id in self.blocks_map.values():
                break

        # TODO blokum co nejsou naucene dat jinou barvu nez zelenou
        self.blocks_list = ListItem(self.scene(), self.rpm, self.m2pix(0.01), 0, 0.18, bdata, self.block_selected_cb, parent=self)

        y = 50
        self.blocks_list.setPos(self.sp, y)
        y += self.blocks_list._height() + self.sp

        self. _place_childs_horizontally(y, self.sp, [self.block_finished_btn, self.block_edit_btn, self.block_on_failure_btn])

        y += self.block_finished_btn._height() + self.sp

        self.block_edit_btn.set_enabled(False)
        self.block_on_failure_btn.set_enabled(False)

        self.h = y
        self.update()

        # items "view"
        self.item_finished_btn = ButtonItem(self.scene(), self.rpm, 0, 0, translate("ProgramItem", "Done"), self, self.item_finished_btn_cb)
        self.item_run_btn = ButtonItem(self.scene(), self.rpm, 0, 0, translate("ProgramItem", "Run"), self, self.item_finished_btn_cb)
        self.item_on_failure_btn = ButtonItem(self.scene(), self.rpm, 0, 0, translate("ProgramItem", "On fail"), self, self.item_on_failure_btn_cb)
        self.item_switch_type_btn = ButtonItem(self.scene(), self.rpm, 0, 0, translate("ProgramItem", "---"), self, self.item_switch_type_btn_cb)
        self.items_list = None

        self.item_finished_btn.set_enabled(False, True)
        self.item_run_btn.set_enabled(False, True)
        self.item_on_failure_btn.set_enabled(False, True)
        self.item_switch_type_btn.set_enabled(False, True)

        self.fixed = False

        self.setZValue(100)

    def get_text_for_item(self, item):

        pose_str = "x=" + str(round(item.place_pose.pose.position.x, 2)) + ", y=" + str(round(item.place_pose.pose.position.y, 2))
        obj = item.object

        if item.place_pose.pose.position.x == 0 and item.place_pose.pose.position.y == 0:
            pose_str = "x=??, y=??"

        if obj == "":

            obj = "??"

        if item.type == ProgIt.GET_READY:
            return QtCore.QCoreApplication.translate("ProgramItem", "get ready")
        elif item.type == ProgIt.MANIP_PICK:

            if item.spec == ProgIt.MANIP_ID:
                return QtCore.QCoreApplication.translate("ProgramItem", "pick object ID=")
            elif item.spec == ProgIt.MANIP_TYPE:
                return QtCore.QCoreApplication.translate("ProgramItem", "pick object type=")

        elif item.type == ProgIt.MANIP_PLACE:

            # TODO pose / polygon
            return QtCore.QCoreApplication.translate("ProgramItem", "place object at ") + pose_str

        elif item.type == ProgIt.MANIP_PICK_PLACE:

            if item.spec == ProgIt.MANIP_ID:
                return QtCore.QCoreApplication.translate("ProgramItem", "pick object") + "'" + obj + "'\n" + QtCore.QCoreApplication.translate("ProgramItem", "place to ") + pose_str
            elif item.spec == ProgIt.MANIP_TYPE:
                return QtCore.QCoreApplication.translate("ProgramItem", "pick object type") + "'" + obj + "'\n" + QtCore.QCoreApplication.translate("ProgramItem", "place to ") + pose_str

        elif item.type == ProgIt.WAIT:

            if item.spec == ProgIt.WAIT_FOR_USER:
                return QtCore.QCoreApplication.translate("ProgramItem", "wait for user")
            elif item.spec == ProgIt.WAIT_UNTIL_USER_FINISHES:
                return QtCore.QCoreApplication.translate("ProgramItem", "wait until user finishes")

    def block_edit_btn_cb(self, btn):

        block_id = self.blocks_map[self.blocks_list.selected_item_idx]

        self.block_finished_btn.set_enabled(False, True)
        self.block_edit_btn.set_enabled(False, True)
        self.block_on_failure_btn.set_enabled(False, True)
        self.blocks_list.set_enabled(False, True)

        idata = []
        self.items_map = {} # map from indexes (key) to item_id (value)

        item_id = self.ph.get_first_item_id(block_id)

        while item_id[0] == block_id:

            imsg = self.ph.get_item_msg(*item_id)

            idata.append("Instruction ID: " + str(item_id[1]) + "\n" + self.get_text_for_item(imsg))
            self.items_map[len(idata)-1] = item_id[1]

            item_id = self.ph.get_id_on_success(*item_id)

            # blok je zacykleny
            if item_id[1] in self.items_map.values():
                break

        self.items_list = ListItem(self.scene(), self.rpm, self.m2pix(0.01), 0, 0.18, idata, self.item_selected_cb, parent=self)
        # TODO disablovat/odlisit itemy co nepotrebuji naucit ?? item_requires_learning

        y = 50
        self.items_list.setPos(self.sp, y)
        y += self.items_list._height() + self.sp

        self. _place_childs_horizontally(y, self.sp, [self.item_finished_btn, self.item_run_btn, self.item_on_failure_btn])

        y += self.item_finished_btn._height() + self.sp

        self. _place_childs_horizontally(y, self.sp, [self.item_switch_type_btn])

        y +=  self.item_switch_type_btn._height() + 3*self.sp

        self.h = y
        self.update()

        # TODO nastavit tlacitka do spravneho stavu
        self.item_finished_btn.set_enabled(True, True)
        self.item_run_btn.set_enabled(True, True)
        self.item_on_failure_btn.set_enabled(True, True)
        self.item_switch_type_btn.set_enabled(True, True)

    def get_selected_block_id(self):

        if self.blocks_list.selected_item_idx is None:
            return None

        return self.blocks_map[self.blocks_list.selected_item_idx]

    def block_selected_cb(self):

        if self.blocks_list.selected_item_idx is not None:

            block_id = self.blocks_map[self.blocks_list.selected_item_idx]

            if self.ph.get_block_on_failure(block_id) != 0:
                self.block_on_failure_btn.set_enabled(True)
            else:
                self.block_on_failure_btn.set_enabled(False)

            self.block_edit_btn.set_enabled(True)

            if self.item_switched_cb is not None:

                self.item_switched_cb(block_id, None)

        else:

            self.block_edit_btn.set_enabled(False)
            self.block_on_failure_btn.set_enabled(False)

    def item_selected_cb(self):

        block_id = self.blocks_map[self.blocks_list.selected_item_idx]

        if  self.items_list.selected_item_idx is not None:

            item_id = self.items_map[self.items_list.selected_item_idx]

            of = self.ph.get_id_on_failure(block_id, item_id)

            if of[0] == block_id and of[1] != 0:
                self.item_on_failure_btn.set_enabled(True)
            else:
                self.item_on_failure_btn.set_enabled(False)

            if self.ph.item_requires_learning(block_id, item_id) and self.ph.item_learned(block_id, item_id):
                self.item_run_btn.set_enabled(True)
            else:
                self.item_run_btn.set_enabled(False)

            # TODO en/dis self.item_switch_type_btn

            if self.item_switched_cb is not None:

                self.item_switched_cb(block_id, item_id)

        else:

            self.item_run_btn.set_enabled(False)
            self.item_on_failure_btn.set_enabled(False)

    def block_on_failure_btn(self, btn):

        pass

    def block_finished_btn_cb(self, btn):

        if self.done_cb is not None:

            self.done_cb()

    def item_finished_btn_cb(self, btn):

        # TODO nastavit tlacitka do spravneho stavu
        self.block_finished_btn.set_enabled(True, True)
        self.block_edit_btn.set_enabled(True, True)
        self.block_on_failure_btn.set_enabled(True, True)
        self.blocks_list.set_enabled(True, True)

        self.item_finished_btn.set_enabled(False, True)
        self.item_run_btn.set_enabled(False, True)
        self.item_on_failure_btn.set_enabled(False, True)
        self.item_switch_type_btn.set_enabled(False, True)

        self.scene().removeItem(self.items_list)
        self.items_list = None

        block_id = self.blocks_map[self.blocks_list.selected_item_idx]

        if self.item_switched_cb is not None:

                self.item_switched_cb(block_id, None)

    def item_on_failure_btn_cb(self, btn):

        pass

    def item_switch_type_btn_cb(self, btn):

        pass

    def boundingRect(self):

        return QtCore.QRectF(0, 0, self.w, self.h)

    def paint(self, painter, option, widget):

        painter.setClipRect(option.exposedRect)
        painter.setRenderHint(QtGui.QPainter.Antialiasing)

        font = QtGui.QFont('Arial', 14)
        painter.setFont(font)

        painter.setPen(QtCore.Qt.white)

        # TODO zmerit velikost textu / pouzit label

        if self.items_list is not None:

            block_id = self.blocks_map[self.blocks_list.selected_item_idx]

            if not self.ph.block_learned(block_id):

                painter.setPen(QtCore.Qt.red)

            painter.drawText(self.sp, 2*self.sp, translate("ProgramItem", "Block") + " ID: " + str(block_id))
        else:

            if not self.ph.program_learned():

                painter.setPen(QtCore.Qt.red)

            painter.drawText(self.sp, 2*self.sp, translate("ProgramItem", "Program") + " ID: " + str(self.ph.get_program_id()))



        painter.setClipRect(option.exposedRect)
        painter.setRenderHint(QtGui.QPainter.Antialiasing)

        pen = QtGui.QPen()
        pen.setStyle(QtCore.Qt.NoPen)
        painter.setPen(pen)

        painter.setBrush(QtCore.Qt.gray)
        painter.setOpacity(0.5)
        painter.drawRoundedRect(QtCore.QRect(0, 0, self.w, self.h), 5.0, 5.0)
