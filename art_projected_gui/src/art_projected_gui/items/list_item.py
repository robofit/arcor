#!/usr/bin/env python

from PyQt4 import QtGui, QtCore
from item import Item
from button_item import ButtonItem
import rospkg

translate = QtCore.QCoreApplication.translate


class ListItem(Item):

    def __init__(self, scene, x, y, w, data, item_selected_cb=None, parent=None):

        self.w = 100
        self.h = 100
        self.sp = 0

        self.item_selected_cb = item_selected_cb

        super(ListItem, self).__init__(scene, x, y, parent=parent)

        self.w = self.m2pix(w)
        self.h = self.m2pix(0.2)
        self.sp = self.m2pix(0.005)

        self.items = []

        self.middle_item_idx = 0
        self.selected_item_idx = None

        for d in data:

            self.items.append(ButtonItem(self.scene(), 0, 0,
                                         d, self, self.item_clicked_cb, width=w, push_button=True))

        rospack = rospkg.RosPack()
        icons_path = rospack.get_path('art_projected_gui') + '/icons/'

        self.up_btn = ButtonItem(self.scene(), 0, 0, translate(
            "ProgramItem", "Up"), self, self.up_btn_cb, width=w, image_path=icons_path + "arrow-up.svg")
        self.down_btn = ButtonItem(self.scene(), 0, 0, translate(
            "ProgramItem", "Down"), self, self.down_btn_cb, width=w, image_path=icons_path + "arrow-down.svg")

        self.up_btn.setPos(0, 0)
        self.down_btn.setPos(0, self.h - self.down_btn.boundingRect().height())

        self.set_current_idx(min(1, len(self.items) - 1))

        self.update()

    def item_clicked_cb(self, btn):

        if not self.isEnabled():

            return

        if not btn.pressed:

            self.selected_item_idx = None

        else:

            self.selected_item_idx = self.items.index(btn)

            for i in range(0, len(self.items)):

                if i != self.selected_item_idx:

                    self.items[i].set_pressed(False)

            self.set_current_idx(self.selected_item_idx)

        if self.item_selected_cb is not None:

            self.item_selected_cb()

    def get_current_idx(self):

        return self.middle_item_idx

    def set_current_idx(self, idx, select=False):

        if select:

            self.selected_item_idx = idx

        self.middle_item_idx = max(idx, min(1, len(self.items) - 1))

        if self.isEnabled():

            if self.middle_item_idx == min(1, len(self.items) - 1):
                self.up_btn.set_enabled(False)
            else:
                self.up_btn.set_enabled(True)

            if idx < len(self.items) - 2:
                self.down_btn.set_enabled(True)
            else:
                self.down_btn.set_enabled(False)

        for it in self.items:

            it.setVisible(False)

            if select:
                it.set_pressed(False)

        # selected item is always vertically centered
        self.items[self.middle_item_idx].setPos(
            0, (self.h - self.items[self.middle_item_idx].boundingRect().height()) / 2)
        self.items[self.middle_item_idx].setVisible(True)

        if select:
            self.items[self.selected_item_idx].set_pressed(True)

        # how much vert. space is used
        vspace = self.items[self.middle_item_idx].boundingRect().height()

        # fill space above middle item
        for idx in range(self.middle_item_idx - 1, -1, -1):

            h = self.items[idx].boundingRect().height()
            y = self.items[idx + 1].y() - self.sp - h

            if y < self.up_btn.y() + self.up_btn.boundingRect().height():
                break

            self.items[idx].setPos(0, y)
            self.items[idx].setVisible(True)
            vspace += self.sp + h

        # fill space below middle item
        for idx in range(self.middle_item_idx + 1, len(self.items)):

            h = self.items[idx].boundingRect().height()
            y = self.items[idx - 1].y() + self.items[idx -
                                                     1].boundingRect().height() + self.sp

            if y + h > self.down_btn.y():
                break

            self.items[idx].setPos(0, y)
            self.items[idx].setVisible(True)
            vspace += self.sp + h

    def up_btn_cb(self, btn):

        if self.middle_item_idx > 0:
            self.set_current_idx(self.middle_item_idx - 1)

    def down_btn_cb(self, btn):

        if self.middle_item_idx < len(self.items) - 1:
            self.set_current_idx(self.middle_item_idx + 1)

    def boundingRect(self):

        return QtCore.QRectF(0, 0, self.w, self.h)

    def paint(self, painter, option, widget):

        pass
