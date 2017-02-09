#!/usr/bin/env python

from PyQt4 import QtGui, QtCore
from item import Item
from button_item import ButtonItem
from list_item import ListItem

translate = QtCore.QCoreApplication.translate


class ProgramListItem(Item):

    def __init__(self, scene, rpm, x, y, program_headers, selected_program_id=None, program_selected_cb=None):

        self.w = 100
        self.h = 100

        self.program_headers = program_headers
        self.program_selected_cb = program_selected_cb

        super(ProgramListItem, self).__init__(scene, rpm, x, y)

        self.w = self.m2pix(0.2)
        self.h = self.m2pix(0.25)

        self.fixed = False
        self.setZValue(100)

        data = []

        for ph in self.program_headers:

            data.append("ID: " + str(ph.id) + "\nName: " + ph.name)

        self.list = ListItem(self.scene(), self.rpm, self.m2pix(0.01), 0, 0.18, data, self.item_selected_cb, parent=self)

        self.run_btn = ButtonItem(self.scene(), self.rpm, 0, 0, translate("ProgramItem", "Run"), self, self.run_btn_cb)
        self.edit_btn = ButtonItem(self.scene(), self.rpm, 0, 0, translate("ProgramItem", "Edit"), self, self.edit_btn_cb)
        self.template_btn = ButtonItem(self.scene(), self.rpm, 0, 0, translate("ProgramItem", "Template"), self, self.template_btn_cb)

        self.run_btn.set_enabled(False)
        self.edit_btn.set_enabled(False)
        self.template_btn.set_enabled(False)

        if selected_program_id is not None:

            for i in range(0, len(self.program_headers)):

                if selected_program_id == self.program_headers[i].id:

                    self.list.set_current_idx(i)
                    break

        sp = self.m2pix(0.01)
        h = 5*sp
        self.list.setPos(sp,  h)
        h += self.list._height()
        h += 2*sp

        self. _place_childs_horizontally(h, sp, [self.run_btn, self.edit_btn, self.template_btn])

        h += self.run_btn._height()
        h += 3*sp

        self.h = h
        self.update()

    def item_selected_cb(self):

        if self.list.selected_item_idx is None:

            self.run_btn.set_enabled(False)
            self.edit_btn.set_enabled(False)
            self.template_btn.set_enabled(False)

        else:

            self.run_btn.set_enabled(True)
            self.edit_btn.set_enabled(True)
            self.template_btn.set_enabled(True)

    def get_current_header(self):

        return self.program_headers[self.list.selected_item_idx]

    def run_btn_cb(self, btn):

        if self.program_selected_cb is not None:
            self.program_selected_cb(self.get_current_header().id, run=True)

    def edit_btn_cb(self, btn):

        if self.program_selected_cb is not None:
            self.program_selected_cb(self.get_current_header().id)

    def template_btn_cb(self, btn):

        if self.program_selected_cb is not None:
            self.program_selected_cb(self.get_current_header().id, template=True)

    def boundingRect(self):

        return QtCore.QRectF(0, 0, self.w, self.h)

    def paint(self, painter, option, widget):

        painter.setClipRect(option.exposedRect)
        painter.setRenderHint(QtGui.QPainter.Antialiasing)

        font = QtGui.QFont('Arial', 14)
        painter.setFont(font)

        painter.setPen(QtCore.Qt.white)

        sp = self.m2pix(0.01)
        painter.drawText(sp, 2*sp, translate("ProgramListItem", "Program list"))

        pen = QtGui.QPen()
        pen.setStyle(QtCore.Qt.NoPen)
        painter.setPen(pen)

        painter.setBrush(QtCore.Qt.gray)
        painter.setOpacity(0.5)
        painter.drawRoundedRect(QtCore.QRect(0, 0, self.w, self.h), 5.0, 5.0)
