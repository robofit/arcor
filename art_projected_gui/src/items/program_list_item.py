#!/usr/bin/env python

from PyQt4 import QtGui, QtCore
from item import Item
from button_item import ButtonItem

translate = QtCore.QCoreApplication.translate

class ProgramHeaderItem(Item):

    def __init__(self, scene, rpm, x, y, w, parent, program_header):

        # TODO empty / learned?

        self.w = w
        self.h = 0

        self.text = ""

        super(ProgramHeaderItem, self).__init__(scene, rpm, x, y, parent)
        self.h = self.m2pix(0.1)

        self.set_header(program_header)
        self.update()

    def set_header(self, ph):

        self.program_header = ph
        self.text = "ID: " + str(self.program_header.id) + "\nName: " + self.program_header.name + "\n" + self.program_header.description
        self.update()

    def boundingRect(self):

        return QtCore.QRectF(0, 0, self.w, self.h)

    def paint(self, painter, option, widget):

        painter.setClipRect(option.exposedRect)
        painter.setRenderHint(QtGui.QPainter.Antialiasing)

        font = QtGui.QFont('Arial', self.get_font_size())
        painter.setFont(font)
        painter.setPen(QtCore.Qt.white)

        painter.drawText(self.boundingRect(), QtCore.Qt.AlignLeft, self.text)

class ProgramListItem(Item):

    def __init__(self, scene, rpm, x, y, program_headers,  program_selected_cb=None):

        self.w = 100
        self.h = 100

        self.program_headers = program_headers
        self.program_selected_cb = program_selected_cb

        super(ProgramListItem, self).__init__(scene, rpm, x, y)

        self.w = self.m2pix(0.2)
        self.h = self.m2pix(0.25)

        self.setZValue(100)

        self.up_btn = ButtonItem(self.scene(), self.rpm, 0, 0, translate("ProgramItem", "Up"), self, self.up_btn_cb)
        self.down_btn = ButtonItem(self.scene(), self.rpm, 0, 0, translate("ProgramItem", "Down"), self, self.down_btn_cb)
        self.run_btn = ButtonItem(self.scene(), self.rpm, 0, 0, translate("ProgramItem", "Run"), self, self.run_btn_cb)
        self.edit_btn = ButtonItem(self.scene(), self.rpm, 0, 0, translate("ProgramItem", "Edit"), self, self.edit_btn_cb)
        self.template_btn = ButtonItem(self.scene(), self.rpm, 0, 0, translate("ProgramItem", "Template"), self, self.template_btn_cb)

        self.set_current_idx(0)
        self.ph = ProgramHeaderItem(self.scene(),  self.rpm,  0, 0,  self.w, self, self.get_current_header())

        sp = self.m2pix(0.005)
        h = sp
        self.up_btn.setPos(sp,  h)
        self.up_btn.set_width(self.w - 2*sp)
        h += self.up_btn.boundingRect().height()
        h += sp
        self.ph.setPos(sp,  h)
        h += self.ph.boundingRect().height()
        h += sp
        self.down_btn.setPos(sp,  h)
        self.down_btn.set_width(self.w - 2*sp)
        h += self.down_btn.boundingRect().height()
        h += 2*sp
        self.run_btn.setPos(sp,  h)
        self.edit_btn.setPos(self.run_btn.x() + self.run_btn.boundingRect().width() + sp,  h)
        self.template_btn.setPos(self.edit_btn.x() + self.edit_btn.boundingRect().width() + sp,  h)
        h += self.run_btn.boundingRect().height()
        h += sp

        self.h = h
        self.update()

    def get_current_header(self):

        return self.program_headers[self.current_program_idx]

    def set_current_idx(self,  idx):

        self.current_program_idx = idx

        if (idx==0):
            self.up_btn.set_enabled(False)
        else:
            self.up_btn.set_enabled(True)

        if (idx < len(self.program_headers)-1):
            self.down_btn.set_enabled(True)
        else:
            self.down_btn.set_enabled(False)

    def up_btn_cb(self):

        if self.current_program_idx > 0:
            self.set_current_idx(self.current_program_idx - 1)
            self.ph.set_header(self.get_current_header())
            self.ph.update()


    def down_btn_cb(self):

        if self.current_program_idx < len(self.program_headers) - 1:
            self.set_current_idx(self.current_program_idx + 1)
            self.ph.set_header(self.get_current_header())
            self.ph.update()

    def run_btn_cb(self):

        if self.program_selected_cb is not None:
            self.program_selected_cb(self.get_current_header().id, run=True)

    def edit_btn_cb(self):

        if self.program_selected_cb is not None:
            self.program_selected_cb(self.get_current_header().id)

    def template_btn_cb(self):

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
        painter.drawRect(0, 0, self.w, self.h)
