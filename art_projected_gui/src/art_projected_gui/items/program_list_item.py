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
        h = 3*sp
        self.list.setPos(sp,  h)
        h += self.list.boundingRect().height()
        h += 2*sp
        isp = (self.boundingRect().width() - (2*sp + self.run_btn.boundingRect().width() + self.edit_btn.boundingRect().width() + self.template_btn.boundingRect().width()))/2
        self.run_btn.setPos(sp,  h)
        self.edit_btn.setPos(self.run_btn.x() + self.run_btn.boundingRect().width() + isp,  h)
        self.template_btn.setPos(self.edit_btn.x() + self.edit_btn.boundingRect().width() + isp,  h)
        h += self.run_btn.boundingRect().height()
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

        pen = QtGui.QPen()
        pen.setStyle(QtCore.Qt.NoPen)
        painter.setPen(pen)

        painter.setBrush(QtCore.Qt.gray)
        painter.setOpacity(0.5)
        # painter.drawRect(0, 0, self.w, self.h)
        painter.drawRoundedRect(QtCore.QRect(0, 0, self.w, self.h), 5.0, 5.0)
