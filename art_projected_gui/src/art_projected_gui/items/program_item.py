#!/usr/bin/env python

from PyQt4 import QtGui, QtCore
from item import Item
from art_msgs.msg import ProgramItem as ProgIt,  LearningRequestGoal
from geometry_msgs.msg import Point32
from button_item import ButtonItem
from art_projected_gui.helpers import conversions
from list_item import ListItem
from art_projected_gui.helpers.items import group_enable, group_visible

translate = QtCore.QCoreApplication.translate


class ProgramItem(Item):

    def __init__(self, scene, x, y, program_helper, done_cb=None, item_switched_cb=None,  learning_request_cb=None):

        self.w = 100
        self.h = 100

        self.done_cb = done_cb
        self.item_switched_cb = item_switched_cb
        self.learning_request_cb = learning_request_cb

        self.readonly = False

        super(ProgramItem, self).__init__(scene, x, y)

        self.w = self.m2pix(0.2)
        self.h = self.m2pix(0.25)
        self.sp = self.m2pix(0.005)

        self.ph = program_helper

        self.block_id = None
        self.item_id = None
        self.ref_id = [3]

        self.block_learned = False
        self.program_learned = False

        # block "view"
        self.block_finished_btn = ButtonItem(self.scene(), 0, 0, translate(
            "ProgramItem", "Done"), self, self.block_finished_btn_cb)
        self.block_edit_btn = ButtonItem(self.scene(), 0, 0, translate(
            "ProgramItem", "Edit"), self, self.block_edit_btn_cb)
        self.block_on_failure_btn = ButtonItem(self.scene(), 0, 0, translate(
            "ProgramItem", "On fail"), self, self.block_on_failure_btn)

        bdata = []

        self.blocks_map = {}  # map from indexes (key) to block_id (value)
        self.blocks_map_rev = {}

        block_id = self.ph.get_first_block_id()

        while block_id != 0:

            bmsg = self.ph.get_block_msg(block_id)

            bdata.append("Block ID: " + str(block_id) + "\nName: " + bmsg.name)
            idx = len(bdata) - 1
            self.blocks_map[idx] = block_id
            self.blocks_map_rev[block_id] = idx

            block_id = self.ph.get_block_on_success(block_id)

            # test for cycle
            if block_id in self.blocks_map_rev:
                break

        self.blocks_list = ListItem(self.scene(), 0, 0, 0.2 - 2 * 0.005, bdata, self.block_selected_cb, parent=self)

        for k, v in self.blocks_map.iteritems():

            self._update_block(v)

        y = 50
        self.blocks_list.setPos(self.sp, y)
        y += self.blocks_list._height() + self.sp

        self. _place_childs_horizontally(y, self.sp, [
                                         self.block_finished_btn, self.block_edit_btn, self.block_on_failure_btn])

        y += self.block_finished_btn._height() + self.sp

        group_enable((self.block_edit_btn, self.block_on_failure_btn), False)

        self.h = y
        self.update()

        # items "view"
        self.item_edit_btn = ButtonItem(self.scene(), 0, 0, translate(
            "ProgramItem", "Edit"), self, self.item_edit_btn_cb)
        self.item_run_btn = ButtonItem(self.scene(), 0, 0, translate(
            "ProgramItem", "Run"), self, self.item_run_btn_cb)
        self.item_on_failure_btn = ButtonItem(self.scene(), 0, 0, translate(
            "ProgramItem", "On fail"), self, self.item_on_failure_btn_cb)

        self.item_finished_btn = ButtonItem(self.scene(), 0, 0, translate(
            "ProgramItem", "Back to blocks"), self, self.item_finished_btn_cb)

        self.items_list = None

        group_visible((self.item_finished_btn, self.item_run_btn,
                       self.item_on_failure_btn, self.item_edit_btn), False)

        # readonly (program running) "view"
        self.pr_pause_btn = ButtonItem(self.scene(), 0, 0, translate(
            "ProgramItem", "Pause"), self, self.pr_pause_btn_cb)
        self.pr_repeat_btn = ButtonItem(self.scene(), 0, 0, translate(
            "ProgramItem", "Repeat"), self, self.pr_repeat_btn_cb)
        self.pr_cancel_btn = ButtonItem(self.scene(), 0, 0, translate(
            "ProgramItem", "Cancel"), self, self.pr_cancel_btn_cb)

        group_visible((self.pr_pause_btn, self.pr_repeat_btn,
                       self.pr_cancel_btn), False)

        self.fixed = False

        self.editing_item = False
        self.edit_request = False
        self.run_request = False

        self.setZValue(100)

    def pr_pause_btn_cb(self, btn):

        pass

    def pr_repeat_btn_cb(self, btn):

        pass

    def pr_cancel_btn_cb(self, btn):

        pass

    def _update_learned(self):

        self.block_learned = self.ph.block_learned(self.block_id)
        self.program_learned = self.ph.program_learned()

    def set_readonly(self, readonly):

        self.readonly = readonly

        if self.readonly:

            if self.items_list is not None:

                self.items_list.setVisible(True)
                self.items_list.setEnabled(False)

            self.blocks_list.set_enabled(False, True)

            group_visible((self.block_finished_btn,
                           self.block_edit_btn, self.block_on_failure_btn), False)
            group_visible((self.pr_pause_btn, self.pr_repeat_btn,
                           self.pr_cancel_btn), True)

        else:

            # TODO
            pass

        self.update()

    def set_active(self, block_id, item_id, ref_id):

        old_block_id = self.block_id

        self.block_id = block_id
        self.item_id = item_id
        self.ref_id = ref_id

        if not self.readonly:

            self.set_readonly(True)

        if old_block_id != self.block_id:

            self._init_items_list()

        self.items_list.set_current_idx(
            self.items_map_rev[self.item_id], select=True)

    def get_text_for_item(self, block_id, item_id):

        item = self.ph.get_item_msg(block_id, item_id)

        text = "(" + str(item.id) + ") "

        if item.type == ProgIt.GET_READY:

            text += QtCore.QCoreApplication.translate(
                "ProgramItem", "GET_READY")

        elif item.type == ProgIt.WAIT_FOR_USER:

            text += QtCore.QCoreApplication.translate(
                "ProgramItem", "WAIT_FOR_USER")

        elif item.type == ProgIt.WAIT_UNTIL_USER_FINISHES:

            text += QtCore.QCoreApplication.translate(
                "ProgramItem", "WAIT_UNTIL_USER_FINISHES")

        elif item.type == ProgIt.PICK_FROM_POLYGON:

            text += QtCore.QCoreApplication.translate(
                "ProgramItem", "PICK_FROM_POLYGON")

            if self.ph.is_object_set(block_id,  item_id):

                text += "\n" + "object type=" + item.object[0]

            else:

                text += "\n" + "object type=??"

        elif item.type == ProgIt.PICK_FROM_FEEDER:

            text += QtCore.QCoreApplication.translate(
                "ProgramItem", "PICK_FROM_FEEDER")

            if self.ph.is_pose_set(block_id, item_id):

                text += "\n" + "x=" + str(round(item.pose[0].pose.position.x, 2)) + ", y=" + str(round(
                    item.pose[0].pose.position.y, 2)) + ", z=" + str(round(item.pose[0].pose.position.z, 2))

            else:

                text += "\n" + "x=??, y=??, z=??"

            if self.ph.is_object_set(block_id,  item_id):

                text += "\n" + "object type=" + item.object[0]

            else:

                text += "\n" + "object type=??"

        elif item.type == ProgIt.PICK_OBJECT_ID:

            text += QtCore.QCoreApplication.translate(
                "ProgramItem", "PICK_OBJECT_ID")

            if self.ph.is_object_set(block_id,  item_id):

                text += "\n" + "object ID=" + item.object[0]

            else:

                text += "\n" + "object ID=??"

        elif item.type == ProgIt.PLACE_TO_POSE:

            text += QtCore.QCoreApplication.translate(
                "ProgramItem", "PLACE_TO_POSE")

            if self.ph.is_object_set(block_id,  item.ref_id[0]):

                ref_item = self.ph.get_item_msg(block_id, item.ref_id[0])

                text += "\n" + \
                    "object from (" + \
                    str(item.ref_id[0]) + ")=" + ref_item.object[0]

            else:

                text += "\n" + "object from (" + str(item.ref_id[0]) + ")=??"

            if self.ph.is_pose_set(block_id, item_id):

                text += "\n" + "x=" + str(round(item.pose[0].pose.position.x, 2)) + ", y=" + str(
                    round(item.pose[0].pose.position.y, 2))

            else:

                text += "\n" + "x=??, y=??"

        elif item.type == ProgIt.PLACE_TO_GRID:

            text += QtCore.QCoreApplication.translate(
                "ProgramItem", "PLACE_TO_GRID")

            if self.ph.is_object_set(block_id,  item_id):

                text += "\n" + "object ID=" + item.object[0]

            else:

                text += "\n" + "object ID=??"

        return text

    def _init_items_list(self):

        idata = []
        self.items_map = {}  # map from indexes (key) to item_id (value)
        self.items_map_rev = {}

        item_id = self.ph.get_first_item_id(self.block_id)

        while item_id[0] == self.block_id:

            idata.append(self.get_text_for_item(*item_id))
            idx = len(idata) - 1
            self.items_map[idx] = item_id[1]
            self.items_map_rev[item_id[1]] = idx

            item_id = self.ph.get_id_on_success(*item_id)

            # test for cycle (e.g. jump from the last item to the first one)
            if item_id[1] in self.items_map_rev:
                break

        self.items_list = ListItem(self.scene(
        ), 0, 0, 0.2 - 2 * 0.005, idata, self.item_selected_cb, parent=self)

        for k, v in self.items_map.iteritems():

            if self.ph.item_requires_learning(self.block_id, v):
                self._update_item(self.block_id, v)
            else:
                self.items_list.items[k].set_enabled(False)

        y = 50
        self.items_list.setPos(self.sp, y)
        y += self.items_list._height() + self.sp

        if not self.readonly:

            self. _place_childs_horizontally(
                y, self.sp, [self.item_edit_btn, self.item_run_btn, self.item_on_failure_btn])

            y += self.item_finished_btn._height() + self.sp

            self. _place_childs_horizontally(
                y, self.sp, [self.item_finished_btn])

            y += self.item_finished_btn._height() + 3 * self.sp

            group_visible((self.item_finished_btn, self.item_run_btn,
                           self.item_on_failure_btn, self.item_edit_btn), True)
            self.item_finished_btn.setEnabled(True)
            group_enable((self.item_run_btn, self.item_on_failure_btn,
                          self.item_on_failure_btn), False)

            group_visible((self.pr_pause_btn, self.pr_repeat_btn,
                           self.pr_cancel_btn), False)

        else:

            self.items_list.setEnabled(False)

            self. _place_childs_horizontally(
                y, self.sp, [self.pr_pause_btn, self.pr_repeat_btn, self.pr_cancel_btn])
            y += self.pr_pause_btn._height() + 3 * self.sp

            pr = (self.pr_pause_btn, self.pr_repeat_btn, self.pr_cancel_btn)
            group_visible(pr, True)
            group_enable(pr, False)

            group_visible((self.item_finished_btn, self.item_run_btn,
                           self.item_on_failure_btn, self.item_edit_btn), False)

        self.h = y
        self.update()

    def block_edit_btn_cb(self, btn):

        group_visible((self.block_finished_btn, self.block_edit_btn,
                       self.block_on_failure_btn, self.blocks_list), False)

        self. _init_items_list()

    def block_selected_cb(self):

        if self.blocks_list.selected_item_idx is not None:

            self.block_id = self.blocks_map[self.blocks_list.selected_item_idx]

            if self.ph.get_block_on_failure(self.block_id) != 0:
                self.block_on_failure_btn.set_enabled(True)
            else:
                self.block_on_failure_btn.set_enabled(False)

            self.block_edit_btn.set_enabled(True)

            if self.item_switched_cb is not None:

                self.item_switched_cb(self.block_id, None)

            self._update_learned()

        else:

            self.block_id = None
            self.item_id = None
            self.block_edit_btn.set_enabled(False)
            self.block_on_failure_btn.set_enabled(False)

    def _handle_item_btns(self):

        of = self.ph.get_id_on_failure(self.block_id, self.item_id)

        if of[0] == self.block_id and of[1] != 0:
            self.item_on_failure_btn.set_enabled(True)
        else:
            self.item_on_failure_btn.set_enabled(False)

        if not self.editing_item:

            if self.ph.item_requires_learning(self.block_id, self.item_id) and self.ph.item_learned(self.block_id, self.item_id):
                self.item_run_btn.set_enabled(True)
            else:
                self.item_run_btn.set_enabled(False)

            self.item_edit_btn.set_enabled(
                self.ph.item_requires_learning(self.block_id, self.item_id))

        else:

            self.item_run_btn.set_enabled(False)

    def item_selected_cb(self):

        if self.items_list.selected_item_idx is not None:

            self.item_id = self.items_map[self.items_list.selected_item_idx]

            self._handle_item_btns()

            if self.item_switched_cb is not None:

                self.item_switched_cb(self.block_id, self.item_id)

            self._update_learned()

        else:

            self.item_id = None
            group_enable(
                (self.item_run_btn, self.item_on_failure_btn, self.item_edit_btn), False)

    def block_on_failure_btn(self, btn):

        # TODO switch to on_failure item
        pass

    def block_finished_btn_cb(self, btn):

        if self.done_cb is not None:

            self.done_cb()

    def item_finished_btn_cb(self, btn):

        # go back to blocks view
        group_visible((self.block_finished_btn, self.block_edit_btn,
                       self.block_on_failure_btn, self.blocks_list), True)
        group_visible((self.item_finished_btn, self.item_run_btn,
                       self.item_on_failure_btn, self.item_edit_btn), False)
        self.block_selected_cb()  # TODO extract method to set buttons to proper state
        self.blocks_list.setEnabled(True)
        self.block_finished_btn.setEnabled(True)

        self.scene().removeItem(self.items_list)
        self.items_list = None
        self.item_id = None

        if self.item_switched_cb is not None:

            self.item_switched_cb(self.block_id, self.item_id)

    def item_on_failure_btn_cb(self, btn):

        pass

    def item_run_btn_cb(self, btn):

        self.run_request = True
        self.set_enabled(False)
        self.learning_request_cb(LearningRequestGoal.EXECUTE_ITEM)

    def item_edit_btn_cb(self, btn):

        self.edit_request = True

        # call action / disable all, wait for result (callback), enable editing
        if not self.editing_item:

            self.learning_request_cb(LearningRequestGoal.GET_READY)

        else:

            self.learning_request_cb(LearningRequestGoal.DONE)

        self.set_enabled(False)

    def learning_request_result(self, success):

        self.set_enabled(True)

        # TODO no success, no editing

        if self.edit_request:

            self.edit_request = False

            if not self.editing_item:

                self.editing_item = True
                self.item_edit_btn.set_caption("Done")
                group_enable((self.item_finished_btn, self.items_list), False)

            else:

                self.editing_item = False
                self.item_edit_btn.set_caption("Edit")
                group_enable((self.item_finished_btn, self.items_list), True)
                self._update_learned()

            self._handle_item_btns()

            if self.item_switched_cb is not None:

                self.item_switched_cb(
                    self.block_id, self.item_id, not self.editing_item)

        elif self.run_request:

            self.run_request = False

    def boundingRect(self):

        return QtCore.QRectF(0, 0, self.w, self.h)

    def set_place_pose(self, x, y,  yaw):

        msg = self.get_current_item()

        msg.pose[0].pose.position.x = x
        msg.pose[0].pose.position.y = y
        msg.pose[0].pose.orientation = conversions.yaw2quaternion(yaw)

        self._update_item()

    def set_pose(self,  ps):

        msg = self.get_current_item()
        msg.pose[0] = ps

        self._update_item()

    def set_object(self, obj):

        msg = self.get_current_item()
        msg.object = [obj]

        self._update_item()

    def set_polygon(self, pts):

        msg = self.get_current_item()

        del msg.polygon[0].polygon.points[:]

        for pt in pts:

            msg.polygon[0].polygon.points.append(Point32(pt[0], pt[1], 0))

        self._update_item()

    # Metoda ulozi 4 body SquareItem-u do place_grid v zprave ProgramItem
    def set_place_grid(self, pts):

        msg = self.get_current_item()

        # predchadzajuce body musime zmazat, aby sme tam nemali body predchadzajucej polohy gridu
        del msg.polygon[0].polygon.points[:]

        for pt in pts:
            msg.polygon[0].polygon.points.append(Point32(pt[0], pt[1], 0))

        self._update_item()


    def _update_block(self, block_id):

        idx = self.blocks_map_rev[block_id]

        if self.ph.block_learned(block_id):
            self.blocks_list.items[idx].set_background_color()
        else:
            self.blocks_list.items[idx].set_background_color(QtCore.Qt.red)

    def _update_item(self, block_id=None, item_id=None):

        if block_id is None and item_id is None:
            block_id = self.block_id
            item_id = self.item_id

        idx = self.items_map_rev[item_id]

        if self.ph.item_learned(block_id, item_id):
            self.items_list.items[idx].set_background_color()
        else:
            self.items_list.items[idx].set_background_color(QtCore.Qt.red)

        self.items_list.items[idx].set_caption(
            self.get_text_for_item(block_id, item_id))

        self._update_block(block_id)

    def get_current_item(self):

        if (self.block_id is not None and self.item_id is not None):

            return self.ph.get_item_msg(self.block_id, self.item_id)

        return None

    def get_ref_item(self):

        if (self.block_id is not None and self.ref_id):

            return self.ph.get_item_msg(self.block_id, self.ref_id[0])
        return None


    def paint(self, painter, option, widget):

        if not self.scene():
            return

        painter.setClipRect(option.exposedRect)
        painter.setRenderHint(QtGui.QPainter.Antialiasing)

        font = QtGui.QFont('Arial', 14)
        painter.setFont(font)

        painter.setPen(QtCore.Qt.white)

        # TODO measure text size / use label

        sp = self.m2pix(0.01)

        if self.block_id is not None:

            if not self.block_learned and not self.readonly:

                painter.setPen(QtCore.Qt.red)

            painter.drawText(sp, 2 * sp, translate("ProgramItem", "Block") + " ID: " + str(self.block_id))
        else:

            if not self.program_learned and not self.readonly:

                painter.setPen(QtCore.Qt.red)

            painter.drawText(sp, 2 * sp, translate("ProgramItem", "Program") + " ID: " + str(self.ph.get_program_id()))

        pen = QtGui.QPen()
        pen.setStyle(QtCore.Qt.NoPen)
        painter.setPen(pen)

        painter.setBrush(QtCore.Qt.gray)
        painter.setOpacity(0.5)
        painter.drawRoundedRect(QtCore.QRect(0, 0, self.w, self.h), 5.0, 5.0)
