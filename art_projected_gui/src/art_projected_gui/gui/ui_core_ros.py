#!/usr/bin/env python

from art_projected_gui.gui import UICore
from PyQt4 import QtCore, QtGui
import rospy
from art_msgs.msg import InstancesArray, UserStatus, InterfaceState, ProgramItem as ProgIt, LearningRequestAction, LearningRequestGoal
from art_projected_gui.items import ObjectItem, ButtonItem, PoseStampedCursorItem, TouchPointsItem, LabelItem, TouchTableItem, ProgramListItem, ProgramItem, DialogItem, PolygonItem
from art_projected_gui.helpers import ProjectorHelper, conversions, error_strings
from art_utils import InterfaceStateManager, ArtApiHelper, ProgramHelper
from art_msgs.srv import TouchCalibrationPoints, TouchCalibrationPointsResponse, NotifyUser, NotifyUserResponse, ProgramErrorResolve, ProgramErrorResolveRequest, startProgram, startProgramRequest
from std_msgs.msg import Empty, Bool
from std_srvs.srv import Trigger, TriggerResponse
from std_srvs.srv import Empty as EmptyService
from geometry_msgs.msg import PoseStamped
import actionlib

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

        super(UICoreRos, self).__init__(
            origin[0], origin[1], size[0], size[1], rpm, port)

        QtCore.QObject.connect(self, QtCore.SIGNAL(
            'objects'), self.object_cb_evt)
        QtCore.QObject.connect(self, QtCore.SIGNAL(
            'user_status'), self.user_status_cb_evt)
        QtCore.QObject.connect(self, QtCore.SIGNAL(
            'interface_state'), self.interface_state_evt)

        QtCore.QObject.connect(self, QtCore.SIGNAL(
            'touch_calibration_points_evt'), self.touch_calibration_points_evt)
        QtCore.QObject.connect(self, QtCore.SIGNAL(
            'touch_detected_evt'), self.touch_detected_evt)
        QtCore.QObject.connect(self, QtCore.SIGNAL(
            'notify_user_evt'), self.notify_user_evt)
        QtCore.QObject.connect(self, QtCore.SIGNAL(
            'learning_request_done_evt'), self.learning_request_done_evt)

        self.user_state = None

        self.program_list = None
        self.program_vis = None
        self.template = False  # TODO this should be stored in program_vis?
        self.last_prog_pos = (0.2, self.height - 0.2)
        self.last_edited_prog_id = None

        self.ph = ProgramHelper()

        cursors = rospy.get_param("~cursors", [])
        for cur in cursors:
            PoseStampedCursorItem(self.scene, cur)

        TouchTableItem(self.scene, '/art/interface/touchtable/touch',
                       list(self.get_scene_items_by_type(PoseStampedCursorItem)))

        self.stop_btn = ButtonItem(self.scene, 0, 0, "STOP",
                                   None, self.stop_btn_clicked, 2.0, QtCore.Qt.red)
        self.stop_btn.setPos(self.scene.width() - self.stop_btn.boundingRect().width() -
                             300, self.scene.height() - self.stop_btn.boundingRect().height() - 60)
        self.stop_btn.set_enabled(True)

        self.projectors = []

        projs = rospy.get_param("~projectors", [])
        for proj in projs:
            self.projectors.append(ProjectorHelper(proj))

        rospy.loginfo("Waiting for /art/brain/learning_request")
        self.learning_action_cl = actionlib.SimpleActionClient(
            '/art/brain/learning_request', LearningRequestAction)
        self.learning_action_cl.wait_for_server()

        self.art = ArtApiHelper()

        self.projectors_calibrated_pub = rospy.Publisher(
            "~projectors_calibrated", Bool, queue_size=1, latch=True)
        self.projectors_calibrated_pub.publish(False)

        self.start_learning_srv = rospy.ServiceProxy(
            '/art/brain/learning/start', startProgram)  # TODO wait for service? where?
        self.stop_learning_srv = rospy.ServiceProxy(
            '/art/brain/learning/stop', Trigger)  # TODO wait for service? where?

        self.program_pause_srv = rospy.ServiceProxy(
            '/art/brain/program/pause', Trigger)

        self.program_resume_srv = rospy.ServiceProxy(
            '/art/brain/program/resume', Trigger)

        self.program_stop_srv = rospy.ServiceProxy(
            '/art/brain/program/stop', Trigger)

        self.emergency_stop_srv = rospy.ServiceProxy(
            '/pr2_ethercat/halt_motors', EmptyService)  # TODO wait for service? where?

        self.emergency_stop_reset_srv = rospy.ServiceProxy(
            '/pr2_ethercat/reset_motors', EmptyService)  # TODO wait for service? where?

        self.robot_halted = None
        self.motors_halted_sub = rospy.Subscriber(
            "/pr2_ethercat/motors_halted", Bool, self.motors_halted_cb)

        QtCore.QObject.connect(self, QtCore.SIGNAL(
            'motors_halted_evt'), self.motors_halted_evt)

        self.program_error_resolve_srv = rospy.ServiceProxy(
            '/art/brain/program/error_response', ProgramErrorResolve)  # TODO wait for service? where?
        self.program_error_dialog = None

        self.grasp_dialog = None

        self.emergency_stopped = False

        rospy.loginfo("Waiting for ART services...")
        self.art.wait_for_api()

        rospy.loginfo("Ready! Starting state machine.")

        # TODO move this to ArtApiHelper ??
        self.obj_sub = rospy.Subscriber(
            '/art/object_detector/object_filtered', InstancesArray, self.object_cb, queue_size=1)
        self.user_status_sub = rospy.Subscriber(
            '/art/user/status', UserStatus, self.user_status_cb, queue_size=1)

        self.touch_points = None
        self.touch_calib_srv = rospy.Service(
            '/art/interface/projected_gui/touch_calibration', TouchCalibrationPoints, self.touch_calibration_points_cb)
        self.notify_user_srv = rospy.Service(
            '/art/interface/projected_gui/notify_user', NotifyUser, self.notify_user_srv_cb)

        proj_calib = True

        if len(self.projectors) > 0:
            rospy.loginfo("Waiting for projector nodes...")
            for proj in self.projectors:
                proj.wait_until_available()
                if not proj.is_calibrated():
                    proj_calib = False

        if proj_calib:

            rospy.loginfo('Projectors already calibrated.')
            self.projectors_calibrated_pub.publish(True)

        else:

            rospy.loginfo('Projectors not calibrated yet - waiting for command...')

        self.projector_calib_srv = rospy.Service(
            '/art/interface/projected_gui/calibrate_projectors', Trigger, self.calibrate_projectors_cb)

        self.state_manager = InterfaceStateManager(
            "PROJECTED UI", cb=self.interface_state_cb)

    def touch_calibration_points_evt(self, pts):

        for it in self.scene.items():

            if isinstance(it, LabelItem):
                continue

            it.setVisible(False)  # TODO remember settings (how?)

        self.notif(translate(
            "UICoreRos", "Touch table calibration started. Please press the white point."), temp=False)
        self.touch_points = TouchPointsItem(self.scene, pts)

    def save_gripper_pose_cb(self, idx):

        topics = ['/art/pr2/right_arm/gripper/pose',
                  '/art/pr2/left_arm/gripper/pose']

        # wait for message, set pose
        try:
            ps = rospy.wait_for_message(topics[idx], PoseStamped, timeout=2)
        except(rospy.ROSException) as e:
            rospy.logerr(str(e))
            self.notif(
                translate("UICoreRos", "Failed to store gripper pose."), temp=False)
            return

        self.notif(translate("UICoreRos", "Gripper pose stored."), temp=False)
        self.program_vis.set_pose(ps)

    def touch_calibration_points_cb(self, req):

        resp = TouchCalibrationPointsResponse()

        pts = []

        for pt in req.points:

            pts.append((pt.point.x, pt.point.y))

        self.emit(QtCore.SIGNAL('touch_calibration_points_evt'), pts)
        self.touched_sub = rospy.Subscriber(
            '/art/interface/touchtable/touch_detected', Empty, self.touch_detected_cb, queue_size=10)
        resp.success = True
        return resp

    def touch_detected_evt(self, msg):

        if self.touch_points is None:
            return

        if not self.touch_points.next():

            self.notif(translate("UICoreRos", "Touch saved."), temp=True)

            for it in self.scene.items():

                if isinstance(it, LabelItem):
                    continue

                # TODO fix this - in makes visible even items that are invisible by purpose
                it.setVisible(True)

            self.notif(
                translate("UICoreRos", "Touch table calibration finished."), temp=False)
            self.scene.removeItem(self.touch_points)
            self.touch_points = None
            self.touched_sub.unregister()

        else:

            self.notif(translate("UICoreRos", "Touch saved."), temp=True)
            self.notif(
                translate("UICoreRos", "Please press the next point."), temp=False)

    def touch_detected_cb(self, msg):

        self.emit(QtCore.SIGNAL('touch_detected_evt'), msg)

    def calibrate_projectors_cb(self, req):

        resp = TriggerResponse()
        resp.success = True

        # call to start_projector_calibration is blocking
        self.proj_calib_timer = rospy.Timer(rospy.Duration(0.001), self.start_projector_calibration, oneshot=True)

        return resp

    def notify_user_srv_cb(self, req):

        self.emit(QtCore.SIGNAL('notify_user_evt'), req)
        return NotifyUserResponse()

    def notify_user_evt(self, req):

        # TODO message should be displayed until user closes it
        if req.duration == rospy.Duration(0):
            self.notif(req.message, message_type=req.type)
        else:
            self.notif(req.message, min_duration=req.duration.to_sec(),
                       temp=True, message_type=req.type)

    def motors_halted_cb(self, msg):

        if self.robot_halted != msg.data:

            self.emit(QtCore.SIGNAL('motors_halted_evt'), msg.data)

        self.robot_halted = msg.data

    def motors_halted_evt(self, halted):

        if not self.emergency_stopped:

            if halted:

                self.notif(translate("UICoreRos", "Robot is halted."))
                self.stop_btn.set_enabled(False)

            else:

                self.notif(translate("UICoreRos", "Robot is up again."))
                self.stop_btn.set_enabled(True)

    def stop_btn_clicked(self, btn):

        try:

            if self.emergency_stopped:
                self.emergency_stop_reset_srv.call()
                self.emergency_stopped = False
                self.stop_btn.set_caption("STOP")
                self.stop_btn.set_background_color(QtCore.Qt.red)
                self.notif(translate("UICoreRos", "Resetting motors"), temp=True)
            else:
                self.emergency_stop_srv.call()
                self.emergency_stopped = True
                self.stop_btn.set_caption("RUN")
                self.stop_btn.set_background_color(QtCore.Qt.green)
                self.notif(
                    translate("UICoreRos", "Emergency stop pressed"), temp=True)

        except rospy.service.ServiceException:

            self.notif(
                translate("UICoreRos", "Failed to stop/run robot."), temp=True)

    def program_error_dialog_cb(self, idx):

        req = ProgramErrorResolveRequest()
        req.user_response_type = idx + 1
        resp = None
        try:
            resp = self.program_error_resolve_srv(req)
        except rospy.ServiceException as e:
            rospy.logerr("Service call failed: " + str(e))

        if resp is None or not resp.success:

            self.notif(
                translate("UICoreRos", "System failure: failed to resolve error."), temp=True)

        self.scene.removeItem(self.program_error_dialog)
        self.program_error_dialog = None

    def interface_state_evt(self, old_state, state, flags):

        system_state_changed = old_state.system_state != state.system_state

        if system_state_changed:
            rospy.logdebug("New system state: " + str(state.system_state) + ", was: " + str(old_state.system_state))
            self.clear_all(True)

        if state.error_severity == InterfaceState.NONE and self.program_error_dialog is not None:

            # hide dialog
            self.scene.removeItem(self.program_error_dialog)
            self.program_error_dialog = None

        # display info/warning/error if there is any - only once (on change)
        if state.error_severity != old_state.error_severity:

            if state.error_severity == InterfaceState.INFO:

                self.notif(translate("UICoreRos", "Error occurred: ") +
                           error_strings.get_error_string(state.error_code), temp=True)

            elif state.error_severity == InterfaceState.WARNING:

                # TODO translate error number to error message
                self.program_error_dialog = DialogItem(self.scene,
                                                       self.width / 2,
                                                       0.1,
                                                       translate(
                                                           "UICoreRos",
                                                           "Handle error: ") + error_strings.get_error_string(
                                                           state.error_code),
                                                       [
                                                           translate(
                                                               "UICoreRos", "Try again"),
                                                           translate(
                                                               "UICoreRos", "Skip instruction"),
                                                           translate(
                                                               "UICoreRos", "Fail instruction"),
                                                           translate(
                                                               "UICoreRos", "End program")
                                                       ],
                                                       self.program_error_dialog_cb)

            # TODO what to do with SEVERE?

        if state.system_state == InterfaceState.STATE_PROGRAM_FINISHED:

            if system_state_changed:

                self.notif(
                    translate("UICoreRos", "The program is done."))

        elif state.system_state == InterfaceState.STATE_IDLE:

            if system_state_changed:

                self.show_program_list()

        elif state.system_state == InterfaceState.STATE_LEARNING:

            self.state_learning(old_state, state, flags, system_state_changed)

        elif state.system_state in [InterfaceState.STATE_PROGRAM_RUNNING, InterfaceState.STATE_PROGRAM_STOPPED]:

            self.state_running(old_state, state, flags, system_state_changed)

    def interface_state_cb(self, old_state, state, flags):

        # print state
        self.emit(QtCore.SIGNAL('interface_state'), old_state, state, flags)

    def state_running(self, old_state, state, flags, system_state_changed):

        if system_state_changed:

            if not self.ph.load(self.art.load_program(state.program_id)):

                self.notif(
                    translate("UICoreRos", "Failed to load program from database."))

                # TODO what to do?
                return

            stopped = state.system_state == InterfaceState.STATE_PROGRAM_STOPPED

            self.show_program_vis(readonly=True, stopped=stopped)

            if stopped:
                self.notif(
                    translate("UICoreRos", "Program paused."), temp=True)

            if not stopped and old_state.system_state == InterfaceState.STATE_PROGRAM_STOPPED:
                self.notif(
                    translate("UICoreRos", "Program resumed."), temp=True)

        # ignore not valid states
        if state.block_id == 0 or state.program_current_item.id == 0:
            rospy.logerr("Invalid state!")
            return

        # TODO if the item id is same - do rather update then clear + add everything?
        self.clear_all()

        self.program_vis.set_active(
            state.block_id, state.program_current_item.id)
        it = state.program_current_item

        if it.type == ProgIt.GET_READY:

            self.notif(translate("UICoreRos", "Robot is getting ready"))

        elif it.type == ProgIt.WAIT_FOR_USER:

            self.notif(translate("UICoreRos", "Waiting for user"))

        elif it.type == ProgIt.WAIT_UNTIL_USER_FINISHES:

            self.notif(
                translate("UICoreRos", "Waiting for user to finish"))

        elif it.type == ProgIt.PICK_FROM_POLYGON:

            obj_id = None
            try:
                obj_id = flags["SELECTED_OBJECT_ID"]
            except KeyError:
                rospy.logerr(
                    "PICK_FROM_POLYGON: SELECTED_OBJECT_ID flag not set")

            if obj_id is not None:
                self.select_object(obj_id)

                obj = self.get_object(obj_id)  # TODO notif - object type
                if obj is not None:
                    self.notif(
                        translate("UICoreRos", "Going to pick object ID ") + obj_id + translate("UICoreRos",
                                                                                                " of type ") + obj.object_type.name + translate(
                            "UICoreRos", " from polygon."))

            self.add_polygon(translate("UICoreRos", "PICK POLYGON"),
                             poly_points=conversions.get_pick_polygon_points(self.ph.get_polygon(state.block_id, it.id)[0]), fixed=True)

        elif it.type == ProgIt.PICK_FROM_FEEDER:

            # TODO PICK_FROM_FEEDER
            pass

        elif it.type == ProgIt.PICK_OBJECT_ID:

            obj_id = self.ph.get_object(state.block_id, it.id)[0][0]

            self.notif(
                translate("UICoreRos", "Picking object with ID=") + obj_id)
            self.select_object(obj_id)

        elif it.type == ProgIt.PLACE_TO_POSE:

            try:
                obj_id = flags["SELECTED_OBJECT_ID"]
            except KeyError:
                rospy.logerr(
                    "PLACE_TO_POSE: SELECTED_OBJECT_ID flag not set")
                return

            obj = self.get_object(obj_id)

            if obj is not None:

                place_pose = self.ph.get_pose(state.block_id, it.id)[0][0]

                self.add_place(translate("UICoreRos", "OBJECT PLACE POSE"),
                               place_pose, obj.object_type, obj_id, fixed=True)

        elif it.type == ProgIt.PLACE_TO_GRID:

            polygons = self.ph.get_polygon(state.block_id, it.id)[0]
            poses = self.ph.get_pose(state.block_id, it.id)[0]
            object_type_name = self.ph.get_object(state.block_id, it.id)[0][0]

            object_type = self.art.get_object_type(object_type_name)

            self.notif(translate("UICoreRos", "Going to place objects into grid"))
            self.add_square(translate("UICoreRos", "PLACE SQUARE GRID"), self.width / 2, self.height / 2, 0.1,
                            0.075, object_type, poses, grid_points=conversions.get_pick_polygon_points(polygons),
                            square_changed=self.square_changed, fixed=True)

        elif it.type == ProgIt.DRILL_POINTS:

            # TODO pres nejaky flag zobrazit kolikata dira se vrta?
            polygons = self.ph.get_polygon(state.block_id, it.id)[0]
            poses = self.ph.get_pose(state.block_id, it.id)[0]

            obj_id = None
            try:
                obj_id = flags["SELECTED_OBJECT_ID"]
            except KeyError:
                rospy.logerr(
                    "DRILL_POINTS: SELECTED_OBJECT_ID flag not set")

            if obj_id is not None:
                self.select_object(obj_id)
                self.notif(translate("UICoreRos", "Going to drill {0} hole(s) into object ID={1}").format(str(len(poses)), obj_id))

            self.add_polygon(translate("UICoreRos", "Objects to be drilled"),
                             poly_points=conversions.get_pick_polygon_points(polygons), fixed=True)

    def show_program_vis(self, readonly=False, stopped=False):

        rospy.logdebug("Showing ProgramItem with readonly=" + str(readonly) + ", stopped=" + str(stopped))
        self.program_vis = ProgramItem(self.scene, self.last_prog_pos[0], self.last_prog_pos[1], self.ph, done_cb=self.learning_done_cb,
                                       item_switched_cb=self.active_item_switched,
                                       learning_request_cb=self.learning_request_cb, stopped=stopped, pause_cb=self.pause_cb, cancel_cb=self.cancel_cb)

        self.program_vis.set_readonly(readonly)

    def pause_cb(self):

        if self.state_manager.state.system_state == InterfaceState.STATE_PROGRAM_STOPPED:

            # TODO call trigger service method
            try:
                resp = self.program_resume_srv()
            except rospy.ServiceException:
                pass

            if resp is not None and resp.success:
                return True
            else:
                self.notif(
                    translate("UICoreRos", "Failed to resume program."), temp=True)
                return False

        elif self.state_manager.state.system_state == InterfaceState.STATE_PROGRAM_RUNNING:

            try:
                resp = self.program_pause_srv()
            except rospy.ServiceException:
                pass

            if resp is not None and resp.success:
                self.notif(
                    translate("UICoreRos", "Program paused."), temp=True)
                return True

            else:

                self.notif(
                    translate("UICoreRos", "Failed to pause program."), temp=True)
                return True

        else:

            rospy.logdebug("Attempt to pause/resume program in strange state: " + str(self.state_manager.state.system_state))
            return False

    def cancel_cb(self):

        if self.state_manager.state.system_state in [InterfaceState.STATE_PROGRAM_RUNNING, InterfaceState.STATE_PROGRAM_STOPPED]:

            try:
                resp = self.program_stop_srv()
            except rospy.ServiceException:
                pass

            if resp is not None and resp.success:
                self.notif(
                    translate("UICoreRos", "Program stopped."), temp=True)
                return True

            else:

                self.notif(
                    translate("UICoreRos", "Failed to stop program."), temp=True)
                return True

        else:

            rospy.logdebug("Attempt to stop program in strange state: " + str(self.state_manager.state.system_state))
            return False

    def clear_all(self, include_dialogs=False):

        rospy.logdebug("Clear all")

        super(UICoreRos, self).clear_all()

        if include_dialogs:

            for it in [self.program_list, self.program_vis]:

                if it is None:
                    continue
                try:
                    self.last_prog_pos = it.get_pos()
                except AttributeError:
                    pass
                break

            for it in [self.program_error_dialog, self.grasp_dialog, self.program_vis, self.program_list]:

                if it is None:
                    continue
                self.remove_scene_items_by_type(type(it))
                it = None

    def state_learning(self, old_state, state, flags, system_state_changed):

        if system_state_changed:

            self.last_edited_prog_id = state.program_id

            if not self.ph.load(self.art.load_program(state.program_id)):

                self.notif(
                    translate("UICoreRos", "Failed to load program from database."))

                # TODO what to do?
                return

            if state.block_id != 0 and state.program_current_item.id != 0:

                # there may be unsaved changes - let's use ProgramItem from brain
                self.ph.set_item_msg(state.block_id, state.program_current_item)

            self.show_program_vis()

        if state.block_id == 0 or state.program_current_item.id == 0:
            rospy.logerr("Invalid state!")
            return

        if old_state.block_id != state.block_id or old_state.program_current_item.id != state.program_current_item.id:
            self.clear_all()

        # TODO overit funkcnost - pokud ma state novejsi timestamp nez nas - ulozit ProgramItem
        if old_state.timestamp == rospy.Time(0) or old_state.timestamp - state.timestamp > rospy.Duration(0):

            rospy.logdebug('Got state with newer timestamp!')
            item = self.ph.get_item_msg(state.block_id, state.program_current_item.id)
            item = state.program_current_item
            self.clear_all()

            self.learning_vis(state.block_id, state.program_current_item.id, not state.edit_enabled)

    def learning_vis(self, block_id, item_id, read_only):

        if not self.ph.item_requires_learning(block_id, item_id):
            self.notif(translate("UICoreRos", "Item has no parameters."))
            return

        self.program_vis.editing_item = not read_only

        # TODO Edit/Done button not visible when there is work in progress!
        if block_id != self.program_vis.block_id or item_id != self.program_vis.item_id:
            self.program_vis.set_active(block_id, item_id)

        msg = self.ph.get_item_msg(block_id, item_id)

        if self.ph.item_learned(block_id, item_id):

            self.notif(
                translate("UICoreRos", "This program item seems to be done"))

        else:

            if msg.type in [ProgIt.PICK_FROM_POLYGON, ProgIt.PICK_FROM_FEEDER,
                            ProgIt.PICK_OBJECT_ID, ProgIt.PLACE_TO_POSE]:
                self.notif(
                    translate("UICoreRos", "Program current manipulation task"))

        if msg.type == ProgIt.PICK_FROM_POLYGON:

            if not self.ph.is_object_set(block_id, item_id):

                self.notif(
                    translate("UICoreRos", "Select object type to be picked up"), temp=True)

            else:

                object_type_name = self.ph.get_object(block_id, item_id)[0][0]
                self.select_object_type(object_type_name)

            if self.ph.is_polygon_set(block_id, item_id):

                polygons = self.ph.get_polygon(block_id, item_id)[0]

                self.add_polygon(translate("UICoreRos", "PICK POLYGON"),
                                 poly_points=conversions.get_pick_polygon_points(polygons), polygon_changed=self.polygon_changed, fixed=read_only)

        elif msg.type == ProgIt.PICK_FROM_FEEDER:

            if self.state_manager.state.edit_enabled and self.grasp_dialog is None:
                self.grasp_dialog = DialogItem(self.scene, self.width / 2, 0.1, "Save gripper pose", [
                    "Right arm", "Left arm"], self.save_gripper_pose_cb)

            if self.ph.is_object_set(block_id, item_id):
                self.select_object_type(self.ph.get_object(block_id, item_id)[0][0])
            else:
                self.notif(
                    translate("UICoreRos", "Select object type to be picked up"), temp=True)

                # TODO show pick pose somehow (arrow??)

        elif msg.type == ProgIt.PICK_OBJECT_ID:
            if self.ph.is_object_set(block_id, item_id):
                self.select_object(self.ph.get_object(block_id, item_id)[0][0])
            else:
                self.notif(
                    translate("UICoreRos", "Select object to be picked up"), temp=True)

        elif msg.type == ProgIt.PLACE_TO_POSE:

            if not self.ph.is_object_set(block_id, item_id):

                (obj_arr, ref_id) = self.ph.get_object(block_id, item_id)

                self.notif(translate(
                    "UICoreRos", "Select object to be picked up in ID=") + str(ref_id))

            else:

                object_type_name = self.ph.get_object(block_id, item_id)[0][0]

                object_type = self.art.get_object_type(object_type_name)
                object_id = None
                self.select_object_type(object_type_name)

                if self.ph.is_pose_set(block_id, item_id):

                    if object_type is not None:
                        self.add_place(translate("UICoreRos", "OBJECT PLACE POSE"),
                                       msg.pose[0], object_type, object_id, place_cb=self.place_pose_changed,
                                       fixed=read_only)
                else:
                    self.notif(
                        translate("UICoreRos", "Set where to place picked object"))
                    self.add_place(translate("UICoreRos", "OBJECT PLACE POSE"), self.get_def_pose(
                    ), object_type, object_id, place_cb=self.place_pose_changed, fixed=read_only)

        elif msg.type == ProgIt.PLACE_TO_GRID:

            object_type_name = self.ph.get_object(block_id, item_id)[0][0]
            poses = self.ph.get_pose(block_id, item_id)[0]
            polygons = self.ph.get_polygon(block_id, item_id)[0]

            object_type = self.art.get_object_type(object_type_name)

            self.notif(translate("UICoreRos", "Place grid"))
            self.add_square(translate("UICoreRos", "PLACE SQUARE GRID"), self.width / 2, self.height / 2, 0.1,
                            0.075, object_type, poses, grid_points=conversions.get_pick_polygon_points(polygons), square_changed=self.square_changed, fixed=read_only)

    def active_item_switched(self, block_id, item_id, read_only=True):

        rospy.logdebug("Program ID:" + str(self.ph.get_program_id()) +
                       ", active item ID: " + str((block_id, item_id)))

        self.clear_all()

        if item_id is None:
            # TODO hlaska
            return

        self.learning_vis(block_id, item_id, read_only)

        self.state_manager.update_program_item(
            self.ph.get_program_id(), block_id, self.ph.get_item_msg(block_id, item_id))

    def get_def_pose(self):

        ps = PoseStamped()
        ps.pose.position.x = self.width / 2
        ps.pose.position.y = self.height / 2
        ps.pose.orientation.w = 1.0
        return ps

    def place_pose_changed(self, place):

        if self.program_vis.editing_item:

            self.program_vis.set_place_pose(place)
            self.state_manager.update_program_item(self.ph.get_program_id(
            ), self.program_vis.block_id, self.program_vis.get_current_item())

    def calib_done_cb(self, proj):

        if proj.is_calibrated():

            self.calib_proj_cnt += 1

            while self.calib_proj_cnt < len(self.projectors):

                if self.projectors[self.calib_proj_cnt].is_calibrated():
                    self.calib_proj_cnt += 1
                    continue

                self.projectors[self.calib_proj_cnt].calibrate(
                    self.calib_done_cb)
                return

            rospy.loginfo('Projectors calibrated.')
            self.projectors_calibrated_pub.publish(True)

        else:

            # calibration failed - let's try again
            rospy.logerr('Calibration failed for projector: ' + proj.proj_id)
            proj.calibrate(self.calib_done_cb)

    def start_projector_calibration(self, evt):

        if len(self.projectors) == 0:

            rospy.loginfo('No projectors to calibrate.')
            self.projectors_calibrated_pub.publish(True)

        else:

            self.projectors_calibrated_pub.publish(False)
            rospy.loginfo('Starting calibration of ' +
                          str(len(self.projectors)) + ' projector(s)')

            self.calib_proj_cnt = 0

            for proj in self.projectors:

                if proj.is_calibrated():

                    self.calib_proj_cnt += 1
                    continue

                else:

                    if not proj.calibrate(self.calib_done_cb):
                        # TODO what to do?
                        rospy.logerr("Failed to start projector calibration")

                    return

            rospy.loginfo('Projectors calibrated.')
            self.projectors_calibrated_pub.publish(True)

    def is_template(self):

        return self.template

    def learning_done_cb(self):

        prog = self.ph.get_program()

        # if it is template - save it with new id
        if self.is_template():

            self.template = False

            headers = self.art.get_program_headers()
            ids = []

            for h in headers:
                ids.append(h.id)

            # is there a better way how to find not used ID for program?
            for i in range(0, 2**16 - 1):
                if i not in ids:
                    prog.header.id = i
                    break
            else:
                rospy.logerr("Failed to find available program ID")

        if not self.art.store_program(prog):

            self.notif(
                translate("UICoreRos", "Failed to store program"), temp=True)
            # TODO what to do?

        self.notif(translate("UICoreRos", "Program stored with ID=") +
                   str(prog.header.id), temp=True)

        self.last_edited_prog_id = prog.header.id

        resp = None
        try:
            resp = self.stop_learning_srv()
        except rospy.ServiceException as e:
            print "Service call failed: %s" % e

        if resp is None or not resp.success:

            rospy.logwarn("Failed to stop learning mode.")
            return

    def program_selected_cb(self, prog_id, run=False, template=False):

        self.template = template

        if run:

            (started, error) = self.art.start_program(prog_id)

            if not started:
                self.notif(translate("UICoreRos", "Failed to start program."))
                rospy.logerr("Brain refused to start program: " + error)
                return

            self.notif(
                translate("UICoreRos", "Starting program ID=" + str(prog_id)), temp=True)
            self.program_list.set_enabled(False)

        else:

            if not self.ph.load(self.art.load_program(prog_id), template):

                self.notif(
                    translate("UICoreRos", "Failed to load program from database."))

                # TODO what to do?
                return

            req = startProgramRequest()
            req.program_id = prog_id
            resp = None
            try:
                resp = self.start_learning_srv(req)
            except rospy.ServiceException as e:
                print "Service call failed: %s" % e

            if resp is None or not resp.success:

                self.notif(
                    translate("UICoreRos", "Failed to start edit mode."))

            # else:
                # self.clear_all()
                # self.show_program_vis()

    def learning_request_cb(self, req):

        if req == LearningRequestGoal.GET_READY:
            self.notif(
                translate("UICoreRos", "Robot is getting ready for learning"))
        elif req == LearningRequestGoal.DONE:

            self.notif(
                translate("UICoreRos", "Robot is getting into default state"))

            if self.grasp_dialog is not None:

                self.scene.removeItem(self.grasp_dialog)
                self.grasp_dialog = None

        elif req == LearningRequestGoal.EXECUTE_ITEM:
            self.notif(
                translate("UICoreRos", "Robot is executing current program instruction"))

        g = LearningRequestGoal()
        g.request = req

        self.learning_action_cl.send_goal(
            g, done_cb=self.learning_request_done_cb, feedback_cb=self.learning_request_feedback_cb)

    def learning_request_feedback_cb(self, fb):

        rospy.logdebug('learning request progress: ' + str(fb.progress))

    def learning_request_done_evt(self, status, result):

        self.program_vis.learning_request_result(result.success)

    def learning_request_done_cb(self, status, result):

        self.emit(QtCore.SIGNAL('learning_request_done_evt'), status, result)

    def show_program_list(self):

        self.notif(translate("UICoreRos", "Please select a program"))

        headers = self.art.get_program_headers()

        d = {}

        headers_to_show = []

        for header in headers:

            ph = ProgramHelper()
            d[header.id] = None

            if ph.load(self.art.load_program(header.id)):

                headers_to_show.append(header)
                d[header.id] = ph.program_learned()

        self.program_list = ProgramListItem(
            self.scene, self.last_prog_pos[0], self.last_prog_pos[1], headers_to_show, d, self.last_edited_prog_id, self.program_selected_cb)

    def object_cb(self, msg):

        self.emit(QtCore.SIGNAL('objects'), msg)

    def object_cb_evt(self, msg):

        for obj_id in msg.lost_objects:

            self.remove_object(obj_id)
            self.notif(translate("UICoreRos", "Object") + " ID=" + str(obj_id) +
                       " " + translate("UICoreRos", "disappeared"), temp=True)

        for inst in msg.instances:

            obj = self.get_object(inst.object_id)

            if obj:
                obj.set_pos(inst.pose.position.x, inst.pose.position.y, inst.pose.position.z)
                obj.set_orientation(conversions.q2a(inst.pose.orientation))
            else:

                obj_type = self.art.get_object_type(inst.object_type)
                self.add_object(inst.object_id, obj_type, inst.pose.position.x, inst.pose.position.y, inst.pose.position.z,
                                conversions.q2a(inst.pose.orientation), self.object_selected)
                self.notif(translate("UICoreRos", "New object") +
                           " ID=" + str(inst.object_id), temp=True)

    def polygon_changed(self, pts):

        if self.program_vis.editing_item:

            self.program_vis.set_polygon(pts)
            self.state_manager.update_program_item(self.ph.get_program_id(
            ), self.program_vis.block_id, self.program_vis.get_current_item())

    '''
        Method which saves grid points and place poses of all objects in grid.
    '''

    def square_changed(self, pts, poses=None):

        self.program_vis.set_place_grid(pts)    # saving grid points into the ProgramItem message
        self.program_vis.set_place_poses(poses)  # saving place poses into the ProgramItem message
        self.state_manager.update_program_item(self.ph.get_program_id(
        ), self.program_vis.block_id, self.program_vis.get_current_item())

    def object_selected(self, id, selected):

        if self.program_vis is None or not self.program_vis.editing_item:
            rospy.logdebug("not in edit mode")
            return False

        msg = self.program_vis.get_current_item()

        if msg is None or len(msg.object) == 0:
            return False

        rospy.logdebug("attempt to select object id: " + id)
        obj = self.get_object(id)

        if msg.type in [ProgIt.PICK_FROM_FEEDER, ProgIt.PICK_FROM_POLYGON]:

            # this type of object is already set
            if len(msg.object) > 0 and obj.object_type.name == msg.object[0]:
                rospy.logdebug("object type " +
                               obj.object_type.name + " already selected")
                return
            else:
                # TODO remove previously inserted polygon, do not insert new
                # place
                rospy.logdebug("selecting new object type: " +
                               obj.object_type.name)
                pass

        if msg.type == ProgIt.PICK_FROM_FEEDER:

            self.program_vis.set_object(obj.object_type.name)
            self.select_object_type(obj.object_type.name)

        elif msg.type == ProgIt.PICK_OBJECT_ID:

            self.program_vis.set_object(obj.object_id)
            self.select_object(obj.object_id)

        elif msg.type == ProgIt.PICK_FROM_POLYGON:

            if obj.object_type.name not in self.selected_object_types:

                self.remove_scene_items_by_type(PolygonItem)

                poly_points = []

                self.program_vis.set_object(obj.object_type.name)
                self.select_object_type(obj.object_type.name)

                for ob in self.get_scene_items_by_type(ObjectItem):
                    if ob.object_type.name != obj.object_type.name:
                        continue
                    poly_points.append(ob.get_pos())

                self.add_polygon(translate("UICoreRos", "PICK POLYGON"),
                                 poly_points, polygon_changed=self.polygon_changed)
                self.notif(
                    translate("UICoreRos", "Check and adjust pick polygon"), temp=True)

        self.state_manager.update_program_item(self.ph.get_program_id(
        ), self.program_vis.block_id, self.program_vis.get_current_item())
        return True

    def user_status_cb(self, msg):

        self.emit(QtCore.SIGNAL('user_status'), msg)

    def user_status_cb_evt(self, msg):

        if msg.user_state != self.user_state:

            if msg.user_state == UserStatus.USER_NOT_CALIBRATED:

                self.notif(translate("UICoreRos", "Please do a calibration pose"))

            elif msg.user_state == UserStatus.USER_CALIBRATED:

                self.notif(translate("UICoreRos", "Successfully calibrated"))

            elif msg.user_state == UserStatus.NO_USER:

                self.notif(translate("UICoreRos", "Waiting for user..."))

        self.user_state = msg
