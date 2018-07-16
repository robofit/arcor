#!/usr/bin/env python

from PyQt4 import QtGui, QtCore
from item import Item
from button_item import ButtonItem
from list_item import ListItem
from desc_item import DescItem
import rospkg

translate = QtCore.QCoreApplication.translate


class ProgramListItem(Item):

    def __init__(
            self,
            scene,
            x,
            y,
            program_headers,
            learned_dict,
            selected_program_id=None,
            program_selected_cb=None,
            program_selection_changed_cb=None):

        self.w = 100
        self.h = 100

        self.program_headers = program_headers
        self.learned_dict = learned_dict
        self.program_selected_cb = program_selected_cb
        self.program_selection_changed_cb = program_selection_changed_cb

        super(ProgramListItem, self).__init__(scene, x, y)

        self.w = self.m2pix(0.2)
        self.h = self.m2pix(0.25)

        self.fixed = False
        self.setZValue(100)

        title = DescItem(self.scene(), 0, 0, self)
        title.set_content(translate("ProgramListItem", "Program list"), 1.2)
        title.setPos(QtCore.QPointF(self.m2pix(0.01), self.m2pix(0.01)))  # TODO it should take coords given to __init__

        data = []
        self.map_from_idx_to_program_id = {}
        self.map_from_program_id_to_idx = {}

        self.program_headers.sort(key=lambda p: p.id)

        for ph in self.program_headers:

            data.append("Program " + str(ph.id) + "\n" + ph.name)
            idx = len(data) - 1
            self.map_from_idx_to_program_id[idx] = ph.id
            self.map_from_program_id_to_idx[ph.id] = idx

        self.list = ListItem(scene, 0, 0, 0.2 - 2 * 0.005, data, self.item_selected_cb, parent=self)

        for idx in range(0, len(data)):

            if not self.learned_dict[self.map_from_idx_to_program_id[idx]]:
                self.list.items[idx].set_background_color(QtCore.Qt.red)

        rospack = rospkg.RosPack()
        icons_path = rospack.get_path('art_projected_gui') + '/icons/'

        self.run_btn = ButtonItem(scene, 0, 0, "BTN", self, self.run_btn_cb, image_path=icons_path + "run.svg")
        self.edit_btn = ButtonItem(scene, 0, 0, "BTN", self, self.edit_btn_cb, image_path=icons_path + "edit.svg")
        self.template_btn = ButtonItem(scene, 0, 0, "BTN", self, self.template_btn_cb, image_path=icons_path + "template.svg")
        self.visualize_btn = ButtonItem(scene, 0, 0, "BTN", self, self.visualize_btn_cb, image_path=icons_path + "visualize.svg")

        self.run_btn.set_enabled(False)
        self.edit_btn.set_enabled(False)
        self.template_btn.set_enabled(False)
        self.visualize_btn.set_enabled(False)

        if selected_program_id is not None:

            self.list.set_current_idx(self.map_from_program_id_to_idx[selected_program_id])

        sp = self.m2pix(0.005)
        # h = title.mapToParent(title.boundingRect().bottomLeft()).y() + sp
        h = 0

        self.list.setPos(sp, h)
        h += self.list._height()
        h += 2 * sp

        btns = (self.run_btn, self.edit_btn, self.template_btn, self.visualize_btn)

        self._place_childs_horizontally(h, sp, btns)
        h += self.run_btn._height()

        h += 3 * sp

        self.h = h

        self.setFlag(QtGui.QGraphicsItem.ItemIsMovable, True)
        self.setFlag(QtGui.QGraphicsItem.ItemIsSelectable, True)

        self.update()

    def item_selected_cb(self):

        if self.list.selected_item_idx is None:

            self.run_btn.set_enabled(False)
            self.edit_btn.set_enabled(False)
            self.template_btn.set_enabled(False)
            self.visualize_btn.set_enabled(False)

            if self.program_selection_changed_cb:
                self.program_selection_changed_cb(None)

        else:

            pid = self.map_from_idx_to_program_id[self.list.selected_item_idx]
            self.run_btn.setEnabled(self.learned_dict[pid])

            for ph in self.program_headers:

                if ph.id == pid:

                    self.edit_btn.set_enabled(not ph.readonly)
                    break

            self.template_btn.set_enabled(True)
            self.visualize_btn.set_enabled(True)

            if self.program_selection_changed_cb:
                self.program_selection_changed_cb(ph.id, ro=ph.readonly, learned=self.learned_dict[ph.id])

    def get_current_header(self):

        return self.program_headers[self.list.selected_item_idx]

    def run_btn_cb(self, btn):

        if self.program_selected_cb is not None:
            self.program_selected_cb(self.get_current_header().id, run=True)

    def edit_btn_cb(self, btn):

        if self.program_selected_cb is not None:
            self.program_selected_cb(self.get_current_header().id)

    def visualize_btn_cb(self, btn):

        if self.program_selected_cb is not None:
            self.program_selected_cb(self.get_current_header().id, visualize=True)

    def template_btn_cb(self, btn):

        if self.program_selected_cb is not None:
            self.program_selected_cb(self.get_current_header().id, template=True)

    def boundingRect(self):

        return QtCore.QRectF(0, 0, self.w, self.h)

    def paint(self, painter, option, widget):

        if not self.scene():
            return

        painter.setClipRect(option.exposedRect)
        painter.setRenderHint(QtGui.QPainter.Antialiasing)

        pen = QtGui.QPen()
        pen.setStyle(QtCore.Qt.NoPen)
        painter.setPen(pen)

        painter.setBrush(QtCore.Qt.gray)
        painter.setOpacity(0.5)
        painter.drawRoundedRect(QtCore.QRect(0, 0, self.w, self.h), 5.0, 5.0)
