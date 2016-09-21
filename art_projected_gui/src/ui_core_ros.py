#!/usr/bin/env python

from ui_core import UICore
from PyQt4 import QtCore
import rospy
from art_msgs.msg import InstancesArray,  UserStatus
from fsm import FSM
from transitions import MachineError
from art_msgs.srv import getProgram,  storeProgram,  startProgram
from art_msgs.msg import ProgramItem as ProgIt
#from button_item import ButtonItem
from object_item import ObjectItem

class UICoreRos(UICore):

    def __init__(self):

        # TODO read x, y, width, height from param server
        super(UICoreRos,  self).__init__(0, 0, 1.2,  0.75)

        self.obj_sub = rospy.Subscriber('/art/object_detector/object_filtered', InstancesArray, self.object_cb, queue_size=1)
        self.user_status_sub = rospy.Subscriber('/art/user/status',  UserStatus,  self.user_status_cb,  queue_size=1)

        QtCore.QObject.connect(self, QtCore.SIGNAL('objects'), self.object_cb_evt)
        QtCore.QObject.connect(self, QtCore.SIGNAL('user_status'), self.user_status_cb_evt)

        self.user_status = None
        self.selected_program_id = None

        self.program_vis.active_item_switched = self.active_item_switched

        self.fsm = FSM()

        # TODO do this automatically??
        # map callbacks from FSM to this instance
        self.fsm.cb_start_calibration = self.cb_start_calibration
        self.fsm.cb_waiting_for_user = self.cb_waiting_for_user
        self.fsm.cb_program_selection = self.cb_program_selection
        self.fsm.cb_waiting_for_user_calibration = self.cb_waiting_for_user_calibration
        self.fsm.cb_learning = self.cb_learning
        self.fsm.is_template = self.is_template

        self.fsm.tr_start()

    def active_item_switched(self):

        print "active_item_switched"

        self.clear_all()

        if self.program_vis.active_item.type in [ProgIt.MANIP_PICK,  ProgIt.MANIP_PLACE,  ProgIt.MANIP_PICK_PLACE]:

            if self.program_vis.active_item.spec == ProgIt.MANIP_TYPE:

                self.select_object_type(self.program_vis.active_item.object)

                # if program item already contains polygon - let's display it
                if len(self.program_vis.active_item.pick_polygon.polygon.points) > 0:

                    poly_points = []

                    for pt in self.program_vis.active_item.pick_polygon.polygon.points:

                        poly_points.append((pt.x,  pt.y))

                    self.add_polygon("PICK POLYGON",  poly_points=poly_points,  polygon_changed=self.polygon_changed)

            else:

                self.select_object(self.program_vis.active_item.object)

            # TODO kdy misto place pose pouzi place polygon? umoznit zmenit pose na polygon a opacne?
            x = self.program_vis.active_item.place_pose.pose.position.x
            y = self.program_vis.active_item.place_pose.pose.position.y

            if  x !=0 and y !=0:
                self.add_place("PLACE POSE",  x,  y,  self.place_pose_changed)
            else:
                self.add_place("PLACE POSE",  self.width/2,  self.height/2,  self.place_pose_changed)

    def place_pose_changed(self,  pos):

        self.program_vis.set_place_pose(pos[0],  pos[1])

    def is_template(self):

        return True

    def cb_learning(self):

        # TODO zobrazit instrukce k tasku
        pass
        #self.buttons.append(ButtonItem()) -> run program

    def cb_start_calibration(self):

        print "calibrating"
        self.fsm.tr_calibrated()

    def cb_waiting_for_user(self):

        self.notif("Waiting for user...")

    def cb_waiting_for_user_calibration(self):

        self.notif("Please do a calibration pose")

    def cb_program_selection(self):

        self.notif("Please select a program")

        # TODO display list of programs -> let user select -> then load it
        self.load_program(0)

        if self.program is not None:

            if self.is_template():

                # just to make sure that template is "clear"
                for it in self.program.items:

                    it.object = ""
                    del it.pick_polygon.polygon.points[:]
                    it.place_pose.pose.position.x = 0
                    it.place_pose.pose.position.y = 0
                    it.place_pose.pose.position.z = 0

            self.program_vis.set_prog(self.program,  self.is_template())
            self.active_item_switched()
            self.fsm.tr_program_selected()
        else:
           self.notif("Loading of requested program failed")

    def load_program(self,  prog_id,  template = False):

        rospy.loginfo('Loading program: ' + str(prog_id))

        try:
            prog_srv = rospy.ServiceProxy('/art/db/program/get', getProgram)
            resp = prog_srv(prog_id)
            self.program = resp.program
        except rospy.ServiceException, e:
            print "Service call failed: %s"%e
            self.program = None

    def object_cb(self,  msg):

        self.emit(QtCore.SIGNAL('objects'),  msg)

    def object_cb_evt(self,  msg):

        for obj_id in msg.lost_objects:

            self.remove_object(obj_id)

        for inst in msg.instances:

            obj = self.get_object(inst.object_id)

            if obj: obj.set_pos(inst.pose.position.x,  inst.pose.position.y)
            else: self.add_object(inst.object_id,  inst.object_type,  inst.pose.position.x,  inst.pose.position.y,  self.object_selected)

    def polygon_changed(self,  pts):

        self.program_vis.set_polygon(pts)

    def object_selected(self,  id,  selected):

        if self.fsm.state != 'learning': return False

        # TODO handle un-selected

        print "selected object: " + str(id)

        obj = self.get_object(id)

        # TODO test typu operace?

        if self.program_vis.active_item.spec == ProgIt.MANIP_TYPE:

            poly_points = []

            self.program_vis.set_object(obj.object_type)
            self.select_object_type(obj.object_type)

            for obj in self.get_scene_items_by_type(ObjectItem):
                poly_points.append(obj.get_pos())

            self.add_polygon("PLACE POLYGON",  poly_points,  polygon_changed=self.polygon_changed)

        elif self.program_vis.active_item.spec == ProgIt.MANIP_ID:

            # TODO
            pass

        return True


    def user_status_cb(self,  msg):

        self.emit(QtCore.SIGNAL('user_status'),  msg)

    def user_status_cb_evt(self,  msg):

        self.user_status = msg

        try:

            if self.user_status.user_state == UserStatus.USER_NOT_CALIBRATED:

                self.fsm.tr_user_present()

            elif self.user_status.user_state == UserStatus.USER_CALIBRATED:

                self.fsm.tr_user_calibrated()

        except MachineError:
            pass
