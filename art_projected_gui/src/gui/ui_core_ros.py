#!/usr/bin/env python

from ui_core import UICore
from PyQt4 import QtCore, QtGui
import rospy
from art_msgs.msg import InstancesArray, UserStatus, InterfaceState, ProgramItem as ProgIt
from fsm import FSM
from transitions import MachineError
from items import ObjectItem, ButtonItem, PoseStampedCursorItem,  TouchPointsItem,  LabelItem,  TouchTableItem, ProgramListItem,  ProgramItem
from helpers import ProjectorHelper,  conversions
from art_utils import InterfaceStateManager,  ArtApiHelper
from art_msgs.srv import TouchCalibrationPoints,  TouchCalibrationPointsResponse,  NotifyUser,  NotifyUserResponse
from std_msgs.msg import Empty
from geometry_msgs.msg import PoseStamped

translate = QtCore.QCoreApplication.translate


class UICoreRos(UICore):

    """The class builds on top of UICore and adds ROS-related stuff and application logic.

    Attributes:
        user_status (UserStatus): current user tracking status
        fsm (FSM): state machine maintaining current state of the interface and proper transitions between states
        state_manager (interface_state_manager): synchronization of interfaces within the ARTable system
        scene_pub (rospy.Publisher): publisher for scene images
        last_scene_update (rospy.Time): time when the last scene image was published
        scene_img_deq (Queue.Queue): thread-safe queue for scene images (which are published in separate thread)
        projectors (list): array of ProjectorHelper instances
        art (ArtApiHelper): easy access to ARTable services

    """

    def __init__(self):

        origin = rospy.get_param("scene_origin")
        size = rospy.get_param("scene_size")
        rpm = rospy.get_param("rpm")
        port = rospy.get_param("scene_server_port")

        super(UICoreRos, self).__init__(origin[0], origin[1], size[0], size[1], rpm,  port)

        QtCore.QObject.connect(self, QtCore.SIGNAL('objects'), self.object_cb_evt)
        QtCore.QObject.connect(self, QtCore.SIGNAL('user_status'), self.user_status_cb_evt)
        QtCore.QObject.connect(self, QtCore.SIGNAL('interface_state'), self.interface_state_evt)

        QtCore.QObject.connect(self, QtCore.SIGNAL('touch_calibration_points_evt'), self.touch_calibration_points_evt)
        QtCore.QObject.connect(self, QtCore.SIGNAL('touch_detected_evt'), self.touch_detected_evt)
        QtCore.QObject.connect(self, QtCore.SIGNAL('notify_user_evt'), self.notify_user_evt)

        self.user_status = None

        self.fsm = FSM()

        # TODO do this automatically??
        # map callbacks from FSM to this instance
        self.fsm.cb_start_calibration = self.cb_start_calibration
        self.fsm.cb_waiting_for_user = self.cb_waiting_for_user
        self.fsm.cb_program_selection = self.cb_program_selection
        self.fsm.cb_waiting_for_user_calibration = self.cb_waiting_for_user_calibration
        self.fsm.cb_learning = self.cb_learning
        self.fsm.cb_running = self.cb_running
        self.fsm.is_template = self.is_template

        self.program_vis = None

        self.state_manager = InterfaceStateManager("PROJECTED UI", cb=self.interface_state_cb)

        cursors = rospy.get_param("~cursors", [])
        for cur in cursors:
            self.scene_items.append(PoseStampedCursorItem(self.scene, self.rpm, cur))

        self.scene_items.append(TouchTableItem(self.scene,  self.rpm, '/art/interface/touchtable/touch', list(self.get_scene_items_by_type(PoseStampedCursorItem))))

        self.scene_items.append(ButtonItem(self.scene, self.rpm, 0, 0, "STOP", None, self.stop_btn_clicked, 2.0, QtCore.Qt.red))
        self.scene_items[-1].setPos(self.scene.width() - self.scene_items[-1].boundingRect().width() - 40, self.scene.height() - self.scene_items[-1].boundingRect().height() - 60)
        self.scene_items[-1].set_enabled(True)

        self.projectors = []

        projs = rospy.get_param("~projectors", [])
        for proj in projs:
            self.add_projector(proj)

        self.art = ArtApiHelper()

    def touch_calibration_points_evt(self,  pts):

        # TODO trigger state change?
        for it in self.scene_items:

            if isinstance(it, LabelItem):
                continue

            it.set_enabled(False, True)

        self.notif(translate("UICoreRos", "Touch table calibration started. Please press the white point."), temp=False)
        self.touch_points = TouchPointsItem(self.scene, self.rpm,  pts)

    def touch_calibration_points_cb(self,  req):

        resp = TouchCalibrationPointsResponse()

        if self.fsm.state not in ['program_selection', 'learning', 'running']:
            resp.success = False
            rospy.logerr('Cannot start touchtable calibration without a user!')
            return resp

        pts = []

        for pt in req.points:

            pts.append((pt.point.x,  pt.point.y))

        self.emit(QtCore.SIGNAL('touch_calibration_points_evt'), pts)
        self.touched_sub = rospy.Subscriber('/art/interface/touchtable/touch_detected',  Empty,  self.touch_detected_cb,  queue_size=10)
        resp.success = True
        return resp

    def touch_detected_evt(self,  msg):

        if self.touch_points is None:
            return

        if not self.touch_points.next():

            self.notif(translate("UICoreRos", "Touch saved."), temp=True)

            for it in self.scene_items:

                if isinstance(it, LabelItem):
                    continue

                it.set_enabled(True, True)

            self.notif(translate("UICoreRos", "Touch table calibration finished."), temp=False)
            self.scene.removeItem(self.touch_points)
            self.touch_points = None
            self.touched_sub.unregister()

        else:

            self.notif(translate("UICoreRos", "Touch saved."), temp=True)
            self.notif(translate("UICoreRos", "Please press the next point."), temp=False)

    def touch_detected_cb(self,  msg):

        self.emit(QtCore.SIGNAL('touch_detected_evt'), msg)

    def start(self):

        rospy.loginfo("Waiting for ART services...")
        self.art.wait_for_api()

        if len(self.projectors) > 0:
            rospy.loginfo("Waiting for projector nodes...")
            for proj in self.projectors:
                proj.wait_until_available()

        rospy.loginfo("Ready! Starting state machine.")

        # TODO move this to ArtApiHelper ??
        self.obj_sub = rospy.Subscriber('/art/object_detector/object_filtered', InstancesArray, self.object_cb, queue_size=1)
        self.user_status_sub = rospy.Subscriber('/art/user/status', UserStatus, self.user_status_cb, queue_size=1)

        self.touch_points = None
        self.touch_calib_srv = rospy.Service('/art/interface/projected_gui/touch_calibration', TouchCalibrationPoints, self.touch_calibration_points_cb)
        self.notify_user_srv = rospy.Service('/art/interface/projected_gui/notify_user', NotifyUser, self.notify_user_srv_cb)

        self.fsm.tr_start()

    def notify_user_srv_cb(self,  req):

        self.emit(QtCore.SIGNAL('notify_user_evt'), req)
        return NotifyUserResponse()

    def notify_user_evt(self,  req):

        if req.duration == rospy.Duration(0):  # TODO message should be displayed until user closes it
            self.notif(req.message,  message_type=req.type)
        else:
            self.notif(req.message,  min_duration=req.duration.to_sec(),  temp=True, message_type=req.type)

    def add_projector(self, proj_id):

        self.projectors.append(ProjectorHelper(proj_id))

    def stop_btn_clicked(self):

        # TODO
        self.notif(translate("UICoreRos", "Emergency stop pressed"), temp=True)

    def interface_state_evt(self, our_state, state, flags):

        if state.system_state == InterfaceState.STATE_PROGRAM_FINISHED:

            self.clear_all()
            self.notif(translate("UICoreRos", "The program is done."), temp=True)
            self.fsm.tr_program_finished()

        elif state.system_state == InterfaceState.STATE_LEARNING:

            # TODO !!
            pass

        elif state.system_state == InterfaceState.STATE_PROGRAM_RUNNING:

            if self.program_vis.prog is None or self.program_vis.prog.header.id != state.program_id:

                program = self.art.load_program(state.program_id)
                if program is not None:
                    self.program_vis.set_prog(program, False)
                else:
                    pass  # TODO error

            if self.fsm.state != 'running':

                self.clear_all()
                self.program_vis.set_running()
                self.fsm.tr_running()

            # TODO updatovat program_vis itemem ze zpravy - pokud se lisi (timestamp) ??
            self.program_vis.set_active(inst_id=state.program_current_item.id)
            it = state.program_current_item

            if our_state.program_current_item.id != state.program_current_item:
                self.clear_all()

            if it.type == ProgIt.GET_READY:

                self.notif(translate("UICoreRos", "Robot is getting ready"))

            elif it.type == ProgIt.WAIT:

                if it.spec == ProgIt.WAIT_FOR_USER:
                    self.notif(translate("UICoreRos", "Waiting for user"))
                elif it.spec == ProgIt.WAIT_UNTIL_USER_FINISHES:
                    self.notif(translate("UICoreRos", "Waiting for user to finish"))

            # TODO MANIP_PICK, MANIP_PLACE
            elif it.type == ProgIt.MANIP_PICK_PLACE:

                obj_id = it.object

                if it.spec == ProgIt.MANIP_ID:

                    self.select_object(it.object)

                elif it.spec == ProgIt.MANIP_TYPE:

                    try:
                        obj_id = flags["SELECTED_OBJECT_ID"]
                    except KeyError:
                        rospy.logerr("MANIP_PICK_PLACE/MANIP_TYPE: SELECTED_OBJECT_ID flag not set")
                        return

                    self.select_object(obj_id)
                    self.add_polygon(translate("UICoreRos", "PICK POLYGON"), poly_points=self.program_vis.active_item.get_pick_polygon_points(),  fixed=True)

                obj = self.get_object(obj_id)
                self.notif(translate("UICoreRos", "Going to manipulate with object ID=") + obj_id)

                if obj is not None:
                    self.add_place(translate("UICoreRos", "OBJECT PLACE POSE"),  it.place_pose, obj.object_type, obj_id,  fixed=True)
                else:
                    # TODO what to do if brain wants to manipulate with non-existent object?
                    pass

    def interface_state_cb(self, our_state, state, flags):

        self.emit(QtCore.SIGNAL('interface_state'), our_state, state, flags)

    # callback from ProgramItem (button press)
    def program_state_changed(self, state):

        if state == 'RUNNING':

            prog = self.program_vis.get_prog()
            prog.header.id = 1

            if not self.art.store_program(prog):
                # TODO what to do?
                self.notif(translate("UICoreRos", "Failed to store program"), temp=True)

            else:

                self.notif(translate("UICoreRos", "Program stored. Starting..."), temp=True)

            # clear all and wait for state update from brain
            self.clear_all()
            self.fsm.tr_program_learned()

            self.art.start_program(prog.header.id)

        # TODO pause / stop -> fsm
        # elif state == ''

        # callback from ProgramItem
    def active_item_switched(self):

        rospy.logdebug("Program ID:" + str(self.program_vis.prog.header.id) + ", active item ID: " + str(self.program_vis.active_item.item.id))

        self.clear_all()
        # TODO block_id
        self.state_manager.update_program_item(self.program_vis.prog.header.id, self.program_vis.prog.blocks[0].id,  self.program_vis.active_item.item)

        if self.program_vis.active_item.item.type in [ProgIt.MANIP_PICK, ProgIt.MANIP_PLACE, ProgIt.MANIP_PICK_PLACE]:

            if self.program_vis.active_item.item_learned():

                self.notif(translate("UICoreRos", "This program item seems to be done"))

            else:

                # TODO vypsat jaky je to task?
                self.notif(translate("UICoreRos", "Program current manipulation task"))

            # TODO loop across program item ids - not indices!!
            idx = self.program_vis.items.index(self.program_vis.active_item)
            if idx > 0:
                for i in range(idx - 1, -1, -1):

                    it = self.program_vis.items[i]

                    if it.item.type in [ProgIt.MANIP_PLACE, ProgIt.MANIP_PICK_PLACE] and it.is_place_pose_set():

                        if it.item.spec == ProgIt.MANIP_ID:
                            obj = self.get_object(it.item.id)
                            self.add_place(translate("UICoreRos", "OBJECT FROM STEP") + " " + str(it.item.id), it.item.place_pose, obj.object_type,  it.item.object, fixed=True)
                        elif it.item.spec == ProgIt.MANIP_TYPE:
                            self.add_place(translate("UICoreRos", "OBJECT FROM STEP") + " " + str(it.item.id), it.item.place_pose, self.art.get_object_type(it.item.object), fixed=True)

                        break

            if self.program_vis.active_item.item.spec == ProgIt.MANIP_TYPE:

                if not self.program_vis.active_item.is_object_set():

                    self.notif(translate("UICoreRos", "Select object type to be picked up"), temp=True)

                else:

                    self.select_object_type(self.program_vis.active_item.item.object)

                # if program item already contains polygon - let's display it
                if self.program_vis.active_item.is_pick_polygon_set():

                    self.add_polygon(translate("UICoreRos", "PICK POLYGON"), poly_points=self.program_vis.active_item.get_pick_polygon_points(), polygon_changed=self.polygon_changed)

            else:

                self.notif(translate("UICoreRos", "Select object to be picked up"), temp=True)
                self.select_object(self.program_vis.active_item.item.object)

            if self.program_vis.active_item.is_object_set():

                # TODO kdy misto place pose pouzi place polygon? umoznit zmenit pose na polygon a opacne?
                if self.program_vis.active_item.item.spec == ProgIt.MANIP_TYPE:
                    object_type = self.art.get_object_type(self.program_vis.active_item.item.object)
                    object_id = None
                else:
                    obj = self.get_object(self.program_vis.active_item.item.object)
                    object_type = obj.object_type
                    object_id = obj.object_id

                if self.program_vis.active_item.is_place_pose_set():

                    self.add_place(translate("UICoreRos", "OBJECT PLACE POSE"), self.program_vis.active_item.get_place_pose(), object_type,  object_id, place_cb=self.place_pose_changed)
                else:
                    self.notif(translate("UICoreRos", "Set where to place picked object"))
                    self.add_place(translate("UICoreRos", "OBJECT PLACE POSE"),  self.get_def_pose(), object_type,  object_id, place_cb=self.place_pose_changed)

    def get_def_pose(self):

        ps = PoseStamped()
        ps.pose.position.x = self.width / 2
        ps.pose.position.y = self.height / 2
        ps.pose.orientation.w = 1.0
        return ps

    def place_pose_changed(self, pos,  yaw):

        self.program_vis.set_place_pose(pos[0], pos[1],  yaw)
        # TODO block_id
        self.state_manager.update_program_item(self.program_vis.prog.header.id, self.program_vis.prog.blocks[0].id,  self.program_vis.active_item.item)

    def cb_running(self):

        pass

    def cb_learning(self):

        # TODO zobrazit instrukce k tasku
        pass

    def calib_done_cb(self, proj):

        if proj.is_calibrated():

            self.calib_proj_cnt += 1

            if self.calib_proj_cnt < len(self.projectors):

                self.projectors[self.calib_proj_cnt].calibrate(self.calib_done_cb)
            else:
                rospy.loginfo('Projectors calibrated.')
                self.fsm.tr_projectors_calibrated()

        else:

            # calibration failed - let's try again
            rospy.logerr('Calibration failed for projector: ' + proj.proj_id)
            proj.calibrate(self.calib_done_cb)

    def cb_start_calibration(self):

        if len(self.projectors) == 0:

            rospy.loginfo('No projectors to calibrate.')
            self.fsm.tr_projectors_calibrated()

        else:

            rospy.loginfo('Starting calibration of ' + str(len(self.projectors)) + ' projector(s)')

            self.calib_proj_cnt = 0

            if not self.projectors[0].calibrate(self.calib_done_cb):
                # TODO what to do?
                rospy.logerr("Failed to start projector calibration")

    def cb_waiting_for_user(self):

        self.notif(translate("UICoreRos", "Waiting for user..."))

    def cb_waiting_for_user_calibration(self):

        self.notif(translate("UICoreRos", "Please do a calibration pose"))

    def is_template(self):

        return True  # TODO implement this

    def program_selected_cb(self,  prog_id,  run=False,  template=False):

        # TODO fix situation when program (template=False) contains "unknown" object

        if run:

            # TODO run program
            pass

        program = self.art.load_program(prog_id)  # TODO option to load program as template?

        if program is not None:

            pos = self.program_list.get_pos()
            self.remove_scene_items_by_type(ProgramListItem)
            self.program_list = None

            self.program_vis = ProgramItem(self.scene, self.rpm, pos[0], pos[1])
            self.program_vis.active_item_switched = self.active_item_switched
            self.program_vis.program_state_changed = self.program_state_changed
            self.scene_items.append(self.program_vis)
            self.program_vis.set_prog(program, template)
            self.active_item_switched()

            self.fsm.tr_program_selected()
        else:
            self.notif(translate("UICoreRos", "Loading of requested program failed"), temp=template)

    def cb_program_selection(self):

        self.notif(translate("UICoreRos", "Please select a program"))

        if self.program_vis is not None:
            pos = self.program_vis.get_pos()
        else:
            pos = (0.2, self.height-0.3)

        self.remove_scene_items_by_type(ProgramItem)
        self.program_vis = None
        self.remove_scene_items_by_type(ProgramListItem)

        self.program_list = ProgramListItem(self.scene, self.rpm, pos[0], pos[1], self.art.get_program_headers(),  self.program_selected_cb)
        self.scene_items.append(self.program_list)

    def object_cb(self, msg):

        self.emit(QtCore.SIGNAL('objects'), msg)

    def object_cb_evt(self, msg):

        for obj_id in msg.lost_objects:

            self.remove_object(obj_id)
            self.notif(translate("UICoreRos", "Object") + " ID=" + str(obj_id) + " " + translate("UICoreRos", "disappeared"), temp=True)

        for inst in msg.instances:

            obj = self.get_object(inst.object_id)

            if obj:
                obj.set_pos(inst.pose.position.x, inst.pose.position.y,  yaw=conversions.quaternion2yaw(inst.pose.orientation))
            else:

                obj_type = self.art.get_object_type(inst.object_type)
                self.add_object(inst.object_id, obj_type, inst.pose.position.x, inst.pose.position.y, conversions.quaternion2yaw(inst.pose.orientation),  self.object_selected)
                self.notif(translate("UICoreRos", "New object") + " ID=" + str(inst.object_id), temp=True)

    def polygon_changed(self, pts):

        self.program_vis.set_polygon(pts)
        # TODO block_id
        self.state_manager.update_program_item(self.program_vis.prog.header.id, self.program_vis.prog.blocks[0].id, self.program_vis.active_item.item)

    def object_selected(self, id, selected):

        if self.fsm.state != 'learning':
            return False
        # TODO handle un-selected

        rospy.logdebug("attempt to select object id: " + id)
        obj = self.get_object(id)

        # TODO test typu operace

        if self.program_vis.active_item.item.spec == ProgIt.MANIP_TYPE:

            # this type of object is already set
            if obj.object_type.name == self.program_vis.active_item.item.object:
                rospy.logdebug("object type " + obj.object_type.name + " already selected")
                return
            else:
                # TODO remove previously inserted polygon, do not insert new place
                rospy.logdebug("selecting new object type: " + obj.object_type.name)
                pass

            poly_points = []

            self.program_vis.set_object(obj.object_type.name)
            self.select_object_type(obj.object_type.name)

            for obj in self.get_scene_items_by_type(ObjectItem):
                poly_points.append(obj.get_pos())

            self.add_polygon(translate("UICoreRos", "PICK POLYGON"), poly_points, polygon_changed=self.polygon_changed)
            self.notif(translate("UICoreRos", "Check and adjust pick polygon"), temp=True)

            self.notif(translate("UICoreRos", "Set where to place picked object"), temp=True)
            self.add_place(translate("UICoreRos", "OBJECT PLACE POSE"), self.get_def_pose(), obj.object_type, place_cb=self.place_pose_changed)

        elif self.program_vis.active_item.item.spec == ProgIt.MANIP_ID:

            # TODO
            pass

        # TODO block_id
        self.state_manager.update_program_item(self.program_vis.prog.header.id, self.program_vis.prog.blocks[0].id, self.program_vis.active_item.item)
        return True

    def user_status_cb(self, msg):

        self.emit(QtCore.SIGNAL('user_status'), msg)

    def user_status_cb_evt(self, msg):

        self.user_status = msg

        try:

            if self.user_status.user_state == UserStatus.USER_NOT_CALIBRATED:

                self.fsm.tr_user_present()

            elif self.user_status.user_state == UserStatus.USER_CALIBRATED:

                self.fsm.tr_user_calibrated()

        except MachineError:
            pass
