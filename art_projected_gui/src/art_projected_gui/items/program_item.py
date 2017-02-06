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

    def __init__(self, scene, rpm, x, y, program_helper, active_item_switched=None):

        self.w = 100
        self.h = 100

        super(ProgramItem, self).__init__(scene, rpm, x, y)

        self.w = self.m2pix(0.2)
        self.h = self.m2pix(0.25)
        self.sp = self.m2pix(0.01)

        self.ph = program_helper

        # block "view"
        self.block_finished_btn = ButtonItem(self.scene(), self.rpm, 0, 0, translate("ProgramItem", "Done"), self, self.block_finished_btn_cb)
        self.block_edit_btn = ButtonItem(self.scene(), self.rpm, 0, 0, translate("ProgramItem", "Edit"), self, self.block_edit_btn_cb)
        self.block_on_failure_btn = ButtonItem(self.scene(), self.rpm, 0, 0, translate("ProgramItem", "On failure"), self, self.block_on_failure_btn)

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
        y += self.blocks_list.boundingRect().height() + self.sp

        isp = (self.boundingRect().width() - (2*self.sp + self.block_finished_btn.boundingRect().width() + self.block_edit_btn.boundingRect().width() + self.block_on_failure_btn.boundingRect().width()))/2

        self.block_finished_btn.setPos(self.sp, y)
        self.block_edit_btn.setPos(self.block_finished_btn.x() + self.block_finished_btn.boundingRect().width() + isp, y)
        self.block_on_failure_btn.setPos(self.block_edit_btn.x() + self.block_edit_btn.boundingRect().width() + isp, y)
        y += self.block_finished_btn.boundingRect().height() + self.sp

        self.block_edit_btn.set_enabled(False)
        self.block_on_failure_btn.set_enabled(False)

        self.h = y
        self.update()

        # items "view"
        self.item_finished_btn = ButtonItem(self.scene(), self.rpm, 0, 0, translate("ProgramItem", "Finished"), self, self.item_finished_btn_cb)
        self.item_on_failure_btn = ButtonItem(self.scene(), self.rpm, 0, 0, translate("ProgramItem", "Go to failure"), self, self.item_on_failure_btn_cb)

        self.item_finished_btn.set_enabled(False, True)
        self.item_on_failure_btn.set_enabled(False, True)

        self.active_item_switched = active_item_switched
        self.fixed = False

        self.setZValue(100)

    def block_edit_btn_cb(self, btn):

        pass

    def block_selected_cb(self):

        if self.blocks_list.selected_item_idx is not None:

            if self.ph.get_block_on_failure(self.blocks_map[self.blocks_list.selected_item_idx]) != 0:
                self.block_on_failure_btn.set_enabled(True)
            self.block_edit_btn.set_enabled(True)

        else:

            self.block_edit_btn.set_enabled(False)
            self.block_on_failure_btn.set_enabled(False)

    def block_on_failure_btn(self, btn):

        pass

    def block_finished_btn_cb(self, btn):

        # TODO skocit na vyber programu - spoustet vzdycky odtud
        pass

    def item_finished_btn_cb(self, btn):

        pass

    def item_on_failure_btn_cb(self, btn):

        pass

    def boundingRect(self):

        return QtCore.QRectF(0, 0, self.w, self.h)

    def paint(self, painter, option, widget):

        painter.setClipRect(option.exposedRect)
        painter.setRenderHint(QtGui.QPainter.Antialiasing)

        font = QtGui.QFont('Arial', 14)
        painter.setFont(font)
        painter.setPen(QtCore.Qt.white)

        painter.drawText(self.sp, 2*self.sp, translate("ProgramItem", "Program") + " ID: " + str(self.ph.get_program_id()))
        # TODO zobrazit jestli uz je nauceny nebo ne

        painter.setClipRect(option.exposedRect)
        painter.setRenderHint(QtGui.QPainter.Antialiasing)

        pen = QtGui.QPen()
        pen.setStyle(QtCore.Qt.NoPen)
        painter.setPen(pen)

        painter.setBrush(QtCore.Qt.gray)
        painter.setOpacity(0.5)
        painter.drawRoundedRect(QtCore.QRect(0, 0, self.w, self.h), 5.0, 5.0)
