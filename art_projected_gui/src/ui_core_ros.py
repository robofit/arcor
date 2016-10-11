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
from art_interface_utils.interface_state_manager import interface_state_manager
from art_msgs.msg import InterfaceState,  InterfaceStateItem
from posestamped_cursor_item import PoseStampedCursorItem

translate = QtCore.QCoreApplication.translate

class UICoreRos(UICore):

    def __init__(self):

        # TODO read x, y, width, height from param server
        super(UICoreRos,  self).__init__(0, 0, 1.2,  0.75)

        self.obj_sub = rospy.Subscriber('/art/object_detector/object_filtered', InstancesArray, self.object_cb, queue_size=1)
        self.user_status_sub = rospy.Subscriber('/art/user/status',  UserStatus,  self.user_status_cb,  queue_size=1)

        QtCore.QObject.connect(self, QtCore.SIGNAL('objects'), self.object_cb_evt)
        QtCore.QObject.connect(self, QtCore.SIGNAL('user_status'), self.user_status_cb_evt)
        QtCore.QObject.connect(self, QtCore.SIGNAL('interface_state'), self.interface_state_evt)


        self.user_status = None
        self.selected_program_id = None

        self.program_vis.active_item_switched = self.active_item_switched
        self.program_vis.program_state_changed = self.program_state_changed

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

        # TODO dodelat integraci state manageru (spis prehodit do ui_code a volat napr z add_polygon apod.)
        self.state_manager = interface_state_manager("PROJECTED UI",  cb=self.interface_state_cb)

        # TODO read/configure inputs from params
        self.scene_items.append(PoseStampedCursorItem(self.scene,  self.rpm,  "left_hand"))
        #self.scene_items.append(PoseStampedCursorItem(self.scene,  self.rpm,  "right_hand"))

    def interface_state_evt(self,  state):

        # TODO !!

        if state.current_syst_state == InterfaceStateItem.STATE_PROGRAM_RUNNING:

            if state.is_clear():

                self.clear_all()

    def interface_state_cb(self,  state):

        self.emit(QtCore.SIGNAL('interface_state'),  state)

    # callback from ProgramItem (button press)
    def program_state_changed(self,  state):

        if state == 'RUNNING':

            prog = self.program_vis.get_prog()
            prog.id = 1

            prog_srv = rospy.ServiceProxy('/art/db/program/store', storeProgram)

            try:
                resp = prog_srv(prog)
            except rospy.ServiceException, e:
                print "Service call failed: %s"%e
                # TODO what to do?
                self.notif(translate("UICoreRos", "Failed to store program"),  temp=True)

            self.notif(translate("UICoreRos", "Program stored. Starting..."),  temp=True)

            self.start_program(prog.id)
            self.fsm.tr_program_learned()

        # TODO pause / stop -> fsm
        #elif state == ''

    # callback from ProgramItem
    def active_item_switched(self):

        rospy.logdebug("Program ID:" + str(self.program_vis.prog.id) + ", active item ID: " + str(self.program_vis.active_item.item.id))

        self.clear_all()
        self.state_manager.clear_all()

        if self.program_vis.active_item.item.type in [ProgIt.MANIP_PICK,  ProgIt.MANIP_PLACE,  ProgIt.MANIP_PICK_PLACE]:

            if self.program_vis.active_item.item_learned():

                self.notif(translate("UICoreRos", "This program item seems to be done"))

            else:

                # TODO vypsat jaky je to task?
                self.notif(translate("UICoreRos", "Program current manipulation task"))

            #self.state_manager.set_syst_state(InterfaceStateItem.STATE_LEARNING,  self.program_vis.prog.id,  self.program_vis.active_item.id)

            # TODO loop across program item ids - not indices!!
            idx = self.program_vis.items.index(self.program_vis.active_item)
            if idx  > 0:
                for i in range(idx-1, -1, -1):

                    it = self.program_vis.items[i]

                    if it.item.type in  [ProgIt.MANIP_PLACE,  ProgIt.MANIP_PICK_PLACE] and it.is_place_pose_set():

                        # TODO add "artif." object instead of place??
                        self.add_place(translate("UICoreRos", "OBJECT FROM STEP") + " " + str(it.item.id),  it.item.place_pose.pose.position.x,  it.item.place_pose.pose.position.y,  fixed=True)
                        break

            if self.program_vis.active_item.item.spec == ProgIt.MANIP_TYPE:

                if not self.program_vis.active_item.is_object_set():

                    self.notif(translate("UICoreRos", "Select object type to be picked up"),  temp=True)

                self.select_object_type(self.program_vis.active_item.item.object)

                # if program item already contains polygon - let's display it
                if self.program_vis.active_item.is_pick_polygon_set():

                    self.add_polygon(translate("UICoreRos", "PICK POLYGON"),  poly_points=self.program_vis.active_item.get_pick_polygon_points(),  polygon_changed=self.polygon_changed)

            else:

                self.notif(translate("UICoreRos", "Select object to be picked up"),  temp=True)
                self.select_object(self.program_vis.active_item.item.object)

            if self.program_vis.active_item.is_object_set():

                # TODO kdy misto place pose pouzi place polygon? umoznit zmenit pose na polygon a opacne?

                if  self.program_vis.active_item.is_place_pose_set():
                    (x,  y) = self.program_vis.active_item.get_place_pose()
                    self.add_place(translate("UICoreRos", "PLACE POSE"),  x,  y,  self.place_pose_changed)
                else:
                    self.notif(translate("UICoreRos", "Set where to place picked object"))
                    self.add_place(translate("UICoreRos", "PLACE POSE"),  self.width/2,  self.height/2,  self.place_pose_changed)

    def place_pose_changed(self,  pos):

        self.program_vis.set_place_pose(pos[0],  pos[1])

    def is_template(self):

        return True

    def cb_learning(self):

        # TODO zobrazit instrukce k tasku
        pass
        #self.buttons.append(ButtonItem()) -> run program

    def calib_done_cb(self,  proj):

        if proj.is_calibrated():

            self.calib_proj_cnt += 1

            if self.calib_proj_cnt < len(self.projectors):

                self.projectors[self.calib_proj_cnt].calibrate(self.calib_done_cb)
            else:
                self.fsm.tr_calibrated()

        else:

            proj.calibrate(self.calib_done_cb)

        if self.calib_proj_cnt == len(self.projectors):

            self.fsm.tr_calibrated()

    def cb_start_calibration(self):

        if len(self.projectors) == 0:

            rospy.logwarn("Nothing to calibrate")
            self.fsm.tr_calibrated()
            return

        print "calibrating"
        self.calib_proj_cnt = 0 # TODO some smarter solution?

        self.projectors[0].calibrate(self.calib_done_cb)

    def cb_waiting_for_user(self):

        self.notif(translate("UICoreRos", "Waiting for user..."))

    def cb_waiting_for_user_calibration(self):

        self.notif(translate("UICoreRos", "Please do a calibration pose"))

    def cb_program_selection(self):

        self.notif(translate("UICoreRos", "Please select a program"))

        # TODO display list of programs -> let user select -> then load it
        self.load_program(0)

        if self.program is not None: # TODO avoid self.program -> duplication

            self.program_vis.set_prog(self.program,  self.is_template())
            self.active_item_switched()
            self.fsm.tr_program_selected()
        else:
           self.notif(translate("UICoreRos", "Loading of requested program failed"),  temp=True)

    def load_program(self,  prog_id,  template = False):

        rospy.loginfo('Loading program: ' + str(prog_id))

        try:
            prog_srv = rospy.ServiceProxy('/art/db/program/get', getProgram)
            resp = prog_srv(prog_id)
            self.program = resp.program
        except rospy.ServiceException, e:
            print "Service call failed: %s"%e
            self.program = None
            return False

        return True

    def start_program(self,  prog_id):

        srv_start = rospy.ServiceProxy('/art/brain/program/start', startProgram)

        try:
            resp = srv_start(prog_id)
        except rospy.ServiceException, e:
            print "Service call failed: %s"%e
            return False

        return True

    def object_cb(self,  msg):

        self.emit(QtCore.SIGNAL('objects'),  msg)

    def object_cb_evt(self,  msg):

        for obj_id in msg.lost_objects:

            self.remove_object(obj_id)
            self.notif(translate("UICoreRos", "Object") + " ID=" + str(obj_id) + " " + translate("UICoreRos", "disappeared"), temp=True)

        for inst in msg.instances:

            obj = self.get_object(inst.object_id)

            if obj: obj.set_pos(inst.pose.position.x,  inst.pose.position.y)
            else:

                self.add_object(inst.object_id,  inst.object_type,  inst.pose.position.x,  inst.pose.position.y,  self.object_selected)
                self.notif(translate("UICoreRos", "New object") + " ID=" + str(inst.object_id),  temp=True)

    def polygon_changed(self,  pts):

        self.program_vis.set_polygon(pts)

    def object_selected(self,  id,  selected):

        if self.fsm.state != 'learning': return False

        # TODO handle un-selected

        print "selected object: " + str(id)

        obj = self.get_object(id)

        # TODO test typu operace

        if self.program_vis.active_item.item.spec == ProgIt.MANIP_TYPE:

            # this type of object is already set
            if obj.object_type == self.program_vis.active_item.item.object: return
            else:
                # TODO remove previously inserted polygon, do not insert new place
                pass

            poly_points = []

            self.program_vis.set_object(obj.object_type)
            self.select_object_type(obj.object_type)

            for obj in self.get_scene_items_by_type(ObjectItem):
                poly_points.append(obj.get_pos())

            self.add_polygon(translate("UICoreRos", "PICK POLYGON"),  poly_points,  polygon_changed=self.polygon_changed)
            self.notif(translate("UICoreRos", "Check and adjust pick polygon"),  temp=True)

            self.notif(translate("UICoreRos", "Set where to place picked object"),  temp=True)
            self.add_place(translate("UICoreRos", "PLACE POSE"),  self.width/2,  self.height/2,  self.place_pose_changed)

        elif self.program_vis.active_item.item.spec == ProgIt.MANIP_ID:

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
