#!/usr/bin/env python

from PyQt4 import QtGui, QtCore
from item import Item
from art_msgs.msg import ProgramItem as ProgIt,  LearningRequestGoal
from geometry_msgs.msg import Point32
from button_item import ButtonItem
from art_projected_gui.helpers import conversions
from list_item import ListItem

translate = QtCore.QCoreApplication.translate


class ProgramItem(Item):

    def __init__(self, scene, rpm, x, y, program_helper, done_cb=None, item_switched_cb=None,  learning_request_cb=None):

        self.w = 100
        self.h = 100

        self.done_cb = done_cb
        self.item_switched_cb = item_switched_cb
        self.learning_request_cb = learning_request_cb

        super(ProgramItem, self).__init__(scene, rpm, x, y)

        self.w = self.m2pix(0.2)
        self.h = self.m2pix(0.25)
        self.sp = self.m2pix(0.01)

        self.ph = program_helper

        self.block_id = None
        self.item_id = None

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

        for k, v in self.blocks_map.iteritems():

            self._update_block(v)

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
        self.item_edit_btn = ButtonItem(self.scene(), self.rpm, 0, 0, translate("ProgramItem", "Edit"), self, self.item_edit_btn_cb)
        self.item_run_btn = ButtonItem(self.scene(), self.rpm, 0, 0, translate("ProgramItem", "Run"), self, self.item_run_btn_cb)
        self.item_on_failure_btn = ButtonItem(self.scene(), self.rpm, 0, 0, translate("ProgramItem", "On fail"), self, self.item_on_failure_btn_cb)

        self.item_finished_btn = ButtonItem(self.scene(), self.rpm, 0, 0, translate("ProgramItem", "Back to blocks"), self, self.item_finished_btn_cb)

        self.items_list = None

        self.item_finished_btn.set_enabled(False, True)
        self.item_run_btn.set_enabled(False, True)
        self.item_on_failure_btn.set_enabled(False, True)
        self.item_edit_btn.set_enabled(False, True)

        self.fixed = False

        self.editing_item = False
        self.edit_request = False
        self.run_request = False

        self.setZValue(100)

    def get_text_for_item(self, item):  # TODO rewrite this

        text = QtCore.QCoreApplication.translate("ProgramItem", "Instruction") + " ID: " + str(item.id) + "\n"

        pose_str = "x=" + str(round(item.place_pose.pose.position.x, 2)) + ", y=" + str(round(item.place_pose.pose.position.y, 2))
        obj = item.object

        if item.place_pose.pose.position.x == 0 and item.place_pose.pose.position.y == 0:
            pose_str = "x=??, y=??"

        if obj == "":

            obj = "??"

        if item.type == ProgIt.GET_READY:
            return text + QtCore.QCoreApplication.translate("ProgramItem", "get ready")
        elif item.type == ProgIt.MANIP_PICK:

            if item.spec == ProgIt.MANIP_ID:
                return text + QtCore.QCoreApplication.translate("ProgramItem", "pick object ID=")
            elif item.spec == ProgIt.MANIP_TYPE:
                return text + QtCore.QCoreApplication.translate("ProgramItem", "pick object type=")

        elif item.type == ProgIt.MANIP_PLACE:

            # TODO pose / polygon
            return text + QtCore.QCoreApplication.translate("ProgramItem", "place object at ") + pose_str

        elif item.type == ProgIt.MANIP_PICK_PLACE:

            if item.spec == ProgIt.MANIP_ID:
                return text + QtCore.QCoreApplication.translate("ProgramItem", "pick object") + "'" + obj + "'\n" + QtCore.QCoreApplication.translate("ProgramItem", "place to ") + pose_str
            elif item.spec == ProgIt.MANIP_TYPE:
                return text + QtCore.QCoreApplication.translate("ProgramItem", "pick object type") + "'" + obj + "'\n" + QtCore.QCoreApplication.translate("ProgramItem", "place to ") + pose_str

        elif item.type == ProgIt.MANIP_PICK_PLACE_FROM_FEEDER:

            return text + QtCore.QCoreApplication.translate("ProgramItem", "pick object type") + "'" + obj + " from feeder'\n" + QtCore.QCoreApplication.translate("ProgramItem", "place to ") + pose_str

        elif item.type == ProgIt.WAIT:

            if item.spec == ProgIt.WAIT_FOR_USER:
                return text + QtCore.QCoreApplication.translate("ProgramItem", "wait for user")
            elif item.spec == ProgIt.WAIT_UNTIL_USER_FINISHES:
                return text + QtCore.QCoreApplication.translate("ProgramItem", "wait until user finishes")

    def block_edit_btn_cb(self, btn):

        self.block_finished_btn.set_enabled(False, True)
        self.block_edit_btn.set_enabled(False, True)
        self.block_on_failure_btn.set_enabled(False, True)
        self.blocks_list.set_enabled(False, True)

        idata = []
        self.items_map = {} # map from indexes (key) to item_id (value)

        item_id = self.ph.get_first_item_id(self.block_id)

        while item_id[0] == self.block_id:

            imsg = self.ph.get_item_msg(*item_id)

            idata.append(self.get_text_for_item(imsg))
            self.items_map[len(idata)-1] = item_id[1]

            item_id = self.ph.get_id_on_success(*item_id)

            # blok je zacykleny
            if item_id[1] in self.items_map.values():
                break

        self.items_list = ListItem(self.scene(), self.rpm, self.m2pix(0.01), 0, 0.18, idata, self.item_selected_cb, parent=self)

        for k, v in self.items_map.iteritems():

            if self.ph.item_requires_learning(self.block_id, v):
                self._update_item(self.block_id, v)
            else:
                self.items_list.items[k].set_enabled(False)  # TODO opravit - list si to nastavuje sam....

        y = 50
        self.items_list.setPos(self.sp, y)
        y += self.items_list._height() + self.sp

        self. _place_childs_horizontally(y, self.sp, [self.item_edit_btn, self.item_run_btn, self.item_on_failure_btn])

        y += self.item_finished_btn._height() + self.sp

        self. _place_childs_horizontally(y, self.sp, [self.item_finished_btn])

        y +=  self.item_finished_btn._height() + 3*self.sp

        self.h = y
        self.update()

        self.item_finished_btn.set_enabled(True, True)
        self.item_run_btn.set_enabled(True, True)
        self.item_run_btn.set_enabled(False)
        self.item_on_failure_btn.set_enabled(True, True)
        self.item_on_failure_btn.set_enabled(False)
        self.item_edit_btn.set_enabled(True, True)
        self.item_edit_btn.set_enabled(False)

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

            self.item_edit_btn.set_enabled(self.ph.item_requires_learning(self.block_id, self.item_id))

        else:

            self.item_run_btn.set_enabled(False)

    def item_selected_cb(self):

        if  self.items_list.selected_item_idx is not None:

            self.item_id = self.items_map[self.items_list.selected_item_idx]

            self._handle_item_btns()

            if self.item_switched_cb is not None:

                self.item_switched_cb(self.block_id, self.item_id)

        else:

            self.item_id = None
            self.item_run_btn.set_enabled(False)
            self.item_on_failure_btn.set_enabled(False)
            self.item_edit_btn.set_enabled(False)

    def block_on_failure_btn(self, btn):

        pass

    def block_finished_btn_cb(self, btn):

        if self.done_cb is not None:

            self.done_cb()

    def item_finished_btn_cb(self, btn):

        # go back to blocks view

        # TODO nastavit tlacitka do spravneho stavu
        self.block_finished_btn.set_enabled(True, True)
        self.block_edit_btn.set_enabled(True, True)
        self.block_on_failure_btn.set_enabled(True, True)
        self.blocks_list.set_enabled(True, True)

        self.item_finished_btn.set_enabled(False, True)
        self.item_run_btn.set_enabled(False, True)
        self.item_on_failure_btn.set_enabled(False, True)
        self.item_edit_btn.set_enabled(False, True)

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

        # TODO call action / disable all, wait for result (callback), enable editing
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
                self.item_finished_btn.set_enabled(False)
                self.items_list.set_enabled(False)

            else:

                self.editing_item = False
                self.item_edit_btn.set_caption("Edit")
                self.item_finished_btn.set_enabled(True)
                self.items_list.set_enabled(True)

            self._handle_item_btns()

            if self.item_switched_cb is not None:

                    self.item_switched_cb(self.block_id, self.item_id, not self.editing_item)

        elif self.run_request:

            self.run_request = False

    def boundingRect(self):

        return QtCore.QRectF(0, 0, self.w, self.h)

    def set_place_pose(self, x, y,  yaw):

        msg = self.get_current_item()

        msg.place_pose.pose.position.x = x
        msg.place_pose.pose.position.y = y
        msg.place_pose.pose.orientation = conversions.yaw2quaternion(yaw)

        self._update_item()

    def set_object(self, obj):

        msg = self.get_current_item()
        msg.object = obj

        self._update_item()

    def set_polygon(self, pts):

        msg = self.get_current_item()

        del msg.pick_polygon.polygon.points[:]

        for pt in pts:

            msg.pick_polygon.polygon.points.append(Point32(pt[0], pt[1], 0))

        self._update_item()

    def _update_block(self, block_id):

        idx = 0

        for k, v in self.blocks_map.iteritems():
            if v == block_id:
                idx = k
                break
        else:
            print "error"
            return

        if self.ph.block_learned(block_id):
            self.blocks_list.items[idx].set_background_color()
        else:
            self.blocks_list.items[idx].set_background_color(QtCore.Qt.red)

    def _update_item(self, block_id=None, item_id=None):

        if block_id is None and item_id is None:
            block_id = self.block_id
            item_id = self.item_id

        idx = 0

        for k, v in self.items_map.iteritems():
            if v == item_id:
                idx = k
                break
        else:
            print "error"
            return

        if self.ph.item_learned(block_id, item_id):
            self.items_list.items[idx].set_background_color()
        else:
            self.items_list.items[idx].set_background_color(QtCore.Qt.red)

        self.items_list.items[idx].set_caption(self.get_text_for_item(self.ph.get_item_msg(block_id, item_id)))

        self._update_block(block_id)

    def get_current_item(self):

        if (self.block_id is not None and self.item_id is not None):

            return self.ph.get_item_msg(self.block_id, self.item_id)

        return None

    def paint(self, painter, option, widget):

        painter.setClipRect(option.exposedRect)
        painter.setRenderHint(QtGui.QPainter.Antialiasing)

        font = QtGui.QFont('Arial', 14)
        painter.setFont(font)

        painter.setPen(QtCore.Qt.white)

        # TODO zmerit velikost textu / pouzit label

        if self.block_id is not None:

            if not self.ph.block_learned(self.block_id):

                painter.setPen(QtCore.Qt.red)

            painter.drawText(self.sp, 2*self.sp, translate("ProgramItem", "Block") + " ID: " + str(self.block_id))
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
