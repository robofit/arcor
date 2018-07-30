#!/usr/bin/env python

from art_projected_gui.gui import UICore
from PyQt4 import QtCore
import rospy
from art_msgs.msg import InstancesArray, InterfaceState, LearningRequestAction,\
    LearningRequestGoal
from art_msgs.msg import HololensState
from art_projected_gui.items import ObjectItem, ButtonItem, PoseStampedCursorItem, TouchPointsItem, LabelItem,\
    TouchTableItem, ProgramListItem, ProgramItem, DialogItem, PolygonItem
from art_projected_gui.helpers import ProjectorHelper, conversions
from art_helpers import InterfaceStateManager, ProgramHelper, ArtRobotHelper, UnknownRobot,\
    RobotParametersNotOnParameterServer
from art_msgs.srv import TouchCalibrationPoints, TouchCalibrationPointsResponse, NotifyUser, NotifyUserResponse,\
    ProgramErrorResolve, ProgramErrorResolveRequest, ProgramIdTrigger, ProgramIdTriggerRequest, NotifyUserRequest
from std_msgs.msg import Empty, Bool
from std_srvs.srv import Trigger, TriggerResponse
from geometry_msgs.msg import PoseStamped
import actionlib
from art_utils import array_from_param, ArtApiHelper
import tf


translate = QtCore.QCoreApplication.translate


class UICoreRos(UICore):

    """The class builds on top of UICore and adds ROS-related stuff and application logic.

    Attributes:
        fsm (FSM): state machine maintaining current state of the interface and proper transitions between states
        state_manager (interface_state_manager): synchronization of interfaces within the ARTable system
        scene_pub (rospy.Publisher): publisher for scene images
        last_scene_update (rospy.Time): time when the last scene image was published
        scene_img_deq (Queue.Queue): thread-safe queue for scene images (which are published in separate thread)
        projectors (list): array of ProjectorHelper instances
        art (ArtApiHelper): easy access to ARTable services

    """

    def __init__(self, instructions_helper, loc):

        self.ih = instructions_helper
        self.loc = loc

        origin = array_from_param("scene_origin", float, 2)
        size = array_from_param("scene_size", float, 2)
        rpm = rospy.get_param("rpm")
        port = rospy.get_param("scene_server_port")

        super(UICoreRos, self).__init__(
            origin[0], origin[1], size[0], size[1], rpm, port, notif_origin=array_from_param("notif_origin", float, 2),
            font_scale=rospy.get_param("font_scale", 1.0))

        self.tfl = tf.TransformListener()

        self.error_dict = {
            InterfaceState.ERROR_ROBOT_HALTED: translate("ErrorStrings", "Robot's motors halted."),
            InterfaceState.ERROR_UNKNOWN: translate("ErrorStrings", "Unknown error."),
            InterfaceState.ERROR_OBJECT_MISSING: translate("ErrorStrings", "Cannot find object."),
            InterfaceState.ERROR_OBJECT_MISSING_IN_POLYGON: translate("ErrorStrings",
                                                                      "There is no object left in the polygon."),
            InterfaceState.ERROR_NO_GRIPPER_AVAILABLE: translate("ErrorStrings", "No gripper available."),
            InterfaceState.ERROR_OBJECT_IN_GRIPPER: translate("ErrorStrings",
                                                              "Robot already holds object and cannot grasp another."),
            InterfaceState.ERROR_NO_OBJECT_IN_GRIPPER: translate("ErrorStrings",
                                                                 "Robot should hold object but it doesn't."),
            InterfaceState.ERROR_PICK_FAILED: translate("ErrorStrings", "Robot failed to pick the object."),
            InterfaceState.ERROR_PLACE_FAILED: translate("ErrorStrings", "Robot failed to place the object."),
            InterfaceState.ERROR_DRILL_FAILED: translate("ErrorStrings", "Robot failed to apply a glue to the object."),
            InterfaceState.ERROR_GRIPPER_NOT_HOLDING_SELECTED_OBJECT: translate("ErrorStrings",
                                                                                "Robot is not holding object."),
        }

        QtCore.QObject.connect(self, QtCore.SIGNAL(
            'objects'), self.object_cb_evt)

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

        self.program_list = None
        self.program_vis = None
        self.template = False  # TODO this should be stored in program_vis?
        self.program_widget_pos = array_from_param("program_widget_pos", float, 2, [0.2, self.height - 0.2])
        self.last_edited_prog_id = None

        self.ph = ProgramHelper()

        cursors = rospy.get_param("~cursors", [])
        for cur in cursors:
            PoseStampedCursorItem(self.scene, cur)

        TouchTableItem(
            self.scene,
            '/art/interface/touchtable/touch',
            list(
                self.get_scene_items_by_type(PoseStampedCursorItem)),
            show_touch_points=rospy.get_param(
                "~show_touch_points",
                False))

        self.projectors = []

        try:

            for proj in array_from_param("~projectors"):
                if len(proj) > 0:
                    self.projectors.append(ProjectorHelper(proj))

        except KeyError:
            pass

        if len(self.projectors) == 0:
            rospy.loginfo("Starting with no projector.")

        rospy.loginfo("Waiting for /art/brain/learning_request")
        self.learning_action_cl = actionlib.SimpleActionClient(
            '/art/brain/learning_request', LearningRequestAction)
        self.learning_action_cl.wait_for_server()

        # HoloLens visualization
        self.start_visualizing_srv = rospy.ServiceProxy(
            '/art/brain/visualize/start', ProgramIdTrigger)  # TODO wait for service? where?
        self.stop_visualizing_srv = rospy.ServiceProxy(
            '/art/brain/visualize/stop', Trigger)  # TODO wait for service? where?
        # for checking if HoloLens is connected
        self.hololens_active_sub = rospy.Subscriber(
            '/art/interface/hololens/active/', Bool, self.hololens_active_cb)
        self.hololens_connected = False
        self.hololens_state_pub = rospy.Publisher(
            '/art/interface/hololens/state', HololensState, queue_size=1)

        self.art = ArtApiHelper()

        self.projectors_calibrated_pub = rospy.Publisher(
            "~projectors_calibrated", Bool, queue_size=1, latch=True)
        self.projectors_calibrated_pub.publish(False)

        self.start_learning_srv = rospy.ServiceProxy(
            '/art/brain/learning/start', ProgramIdTrigger)  # TODO wait for service? where?
        self.stop_learning_srv = rospy.ServiceProxy(
            '/art/brain/learning/stop', Trigger)  # TODO wait for service? where?

        self.visualizing = False

        self.program_pause_srv = rospy.ServiceProxy(
            '/art/brain/program/pause', Trigger)

        self.program_resume_srv = rospy.ServiceProxy(
            '/art/brain/program/resume', Trigger)

        self.program_stop_srv = rospy.ServiceProxy(
            '/art/brain/program/stop', Trigger)

        self.robot_halted = None
        # TODO read status of robot from InterfaceState
        self.motors_halted_sub = rospy.Subscriber(
            "/pr2_ethercat/motors_halted", Bool, self.motors_halted_cb)

        QtCore.QObject.connect(self, QtCore.SIGNAL(
            'motors_halted_evt'), self.motors_halted_evt)

        self.program_error_resolve_srv = rospy.ServiceProxy(
            '/art/brain/program/error_response', ProgramErrorResolve)  # TODO wait for service? where?
        self.program_error_dialog = None

        rospy.loginfo("Waiting for ART services...")
        self.art.wait_for_api()

        # TODO move this to ArtApiHelper ??
        self.obj_sub = rospy.Subscriber(
            '/art/object_detector/object_filtered', InstancesArray, self.object_cb, queue_size=1)

        self.touch_points = None
        self.touch_calib_srv = rospy.Service(
            '/art/interface/projected_gui/touch_calibration', TouchCalibrationPoints, self.touch_calibration_points_cb)
        self.notify_user_srv = rospy.Service(
            '/art/interface/projected_gui/notify_user', NotifyUser, self.notify_user_srv_cb)
        try:
            self.rh = ArtRobotHelper()
        except UnknownRobot:
            rospy.logerr("Unknown robot")
        except RobotParametersNotOnParameterServer:
            rospy.logerr("Robot parameters not on parameters server")
            # TODO: what to do? wait until it is loaded?

        self.robot_arms = self.rh.get_robot_arms()

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

        self.current_instruction = None
        self.items_to_keep = []
        self.vis_instructions = []

        self.items_to_keep_timer = QtCore.QTimer()
        self.items_to_keep_timer.timeout.connect(self.items_to_keep_timer_tick)
        self.items_to_keep_timer.start(100)

        self.state_manager = InterfaceStateManager(
            "PROJECTED UI", cb=self.interface_state_cb)

        # TODO show some icon instead
        self.user_pres_notif = LabelItem(self.scene, 0.05, 0.08, 0.2, 0.03)
        QtCore.QObject.connect(self, QtCore.SIGNAL(
            'user_pres_evt'), self.user_pres_evt)
        self.user_pres = None
        self.user_pres_sub = rospy.Subscriber("/art/interface/user/present", Bool, self.user_pres_cb)

        rospy.loginfo("Projected GUI ready!")

    def user_pres_cb(self, msg):

        self.emit(QtCore.SIGNAL('user_pres_evt'), msg)

    def user_pres_evt(self, msg):

        if self.user_pres is None or msg.data != self.user_pres:
            self.user_pres = msg.data
            if self.user_pres:
                self.user_pres_notif.add_msg("User detected", NotifyUserRequest.INFO)
            else:
                self.user_pres_notif.add_msg("User not present", NotifyUserRequest.INFO)

    def items_to_keep_timer_tick(self):

        now = rospy.Time.now()

        to_delete = []

        for item, ts in self.items_to_keep:

            if ts < now:
                self.scene.removeItem(item)
                to_delete.append((item, ts))

        if to_delete:
            rospy.loginfo("Deleting " + str(len(to_delete)) + " scene item(s).")

        for td in to_delete:
            self.items_to_keep.remove(td)

    def get_error_string(self, error):

        if error not in self.error_dict:
            rospy.logdebug("Undefined error: " + str(error))
            return "Undefined error"

        return self.error_dict[error]

    def touch_calibration_points_evt(self, pts):

        for it in self.scene.items():

            if isinstance(it, LabelItem):
                continue

            it.setVisible(False)  # TODO remember settings (how?)

        self.notif(translate(
            "UICoreRos", "Touch table calibration started. Please press the white point."))
        self.touch_points = TouchPointsItem(self.scene, pts)

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

        if halted:

            self.notif(translate("UICoreRos", "Robot is halted."))

        else:

            self.notif(translate("UICoreRos", "Robot is up again."), temp=True)

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
                translate(
                    "UICoreRos",
                    "System failure: failed to resolve error."),
                temp=True,
                message_type=NotifyUserRequest.ERROR)

        self.scene.removeItem(self.program_error_dialog)
        self.program_error_dialog = None

        if self.program_vis:
            self.program_vis.set_program_btns_enabled(True)

    def interface_state_evt(self, old_state, state, flags):

        system_state_changed = old_state.system_state != state.system_state

        if system_state_changed:
            rospy.logdebug("New system state: " + str(state.system_state) + ", was: " + str(old_state.system_state))
            self.clear_all(True)

        if state.error_severity == InterfaceState.NONE and self.program_error_dialog is not None:

            # hide dialog
            self.scene.removeItem(self.program_error_dialog)
            self.program_error_dialog = None
            if self.program_vis:
                self.program_vis.set_program_btns_enabled(True)

        # display info/warning/error if there is any - only once (on change)
        if state.error_severity != old_state.error_severity:

            if state.error_severity == InterfaceState.INFO:

                self.notif(self.get_error_string(state.error_code), message_type=NotifyUserRequest.ERROR, temp=True)

            elif state.error_severity == InterfaceState.WARNING:

                if state.system_state == InterfaceState.STATE_LEARNING:

                    self.notif(self.get_error_string(state.error_code), temp=True,
                               message_type=NotifyUserRequest.ERROR)

                else:

                    if self.program_vis:
                        self.program_vis.set_program_btns_enabled(False)

                    # TODO translate error number to error message
                    self.program_error_dialog = DialogItem(self.scene,
                                                           self.width / 2,
                                                           0.1,
                                                           self.get_error_string(
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

                    self.notif(translate("UICoreRos", "Please resolve error using dialog."),
                               message_type=NotifyUserRequest.ERROR)

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

        elif state.system_state == InterfaceState.STATE_VISUALIZE:

            self.state_visualizing(old_state, state, flags, system_state_changed)

    def interface_state_cb(self, old_state, state, flags):

        # print state
        self.emit(QtCore.SIGNAL('interface_state'), old_state, state, flags)

    def state_running(self, old_state, state, flags, system_state_changed):

        if system_state_changed:

            if not self.ph.load(self.art.load_program(state.program_id)):

                self.notif(
                    translate(
                        "UICoreRos",
                        "Failed to load program from database."),
                    message_type=NotifyUserRequest.ERROR)

                # TODO what to do?
                return

            stopped = state.system_state == InterfaceState.STATE_PROGRAM_STOPPED

            self.show_program_vis(readonly=True, stopped=stopped, running=True)

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

        it = state.program_current_item

        if it.type in self.ih.known_instructions():

            self.current_instruction = self.ih[it.type].gui.run(self, state.block_id,
                                                                state.program_current_item.id, flags=flags)

            self.program_vis.instruction = self.current_instruction  # TODO how to avoid this?
            self.program_vis.set_active(
                state.block_id, state.program_current_item.id)

        else:

            # TODO big error!
            rospy.logfatal("Unsupported instruction!")

    def show_program_vis(self, readonly=False, stopped=False, running=False, visualize=False):

        if not running:
            item_switched_cb = self.active_item_switched
        else:
            item_switched_cb = None

        if visualize:
            item_switched_cb = self.active_item_switched_for_visualization

        rospy.logdebug("Showing ProgramItem with readonly=" + str(readonly) + ", stopped=" + str(stopped))
        self.program_vis = ProgramItem(
            self.scene,
            self.program_widget_pos[0],
            self.program_widget_pos[1],
            self.ph,
            self.current_instruction,
            self.ih,
            done_cb=self.learning_done_cb,
            item_switched_cb=item_switched_cb,
            learning_request_cb=self.learning_request_cb,
            stopped=stopped,
            pause_cb=self.pause_cb,
            cancel_cb=self.cancel_cb,
            visualize=visualize,
            v_visualize_cb=self.v_visualize_cb,
            v_back_cb=self.v_back_cb,
            vis_pause_cb=self.vis_pause_cb,
            vis_stop_cb=self.vis_stop_cb,
            vis_replay_cb=self.vis_replay_cb,
            vis_back_to_blocks_cb=self.vis_back_to_blocks_cb)

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
                self.notif(translate("UICoreRos", "Failed to resume program."),
                           temp=True, message_type=NotifyUserRequest.ERROR)
                return False

        elif self.state_manager.state.system_state == InterfaceState.STATE_PROGRAM_RUNNING:

            try:
                resp = self.program_pause_srv()
            except rospy.ServiceException:
                pass

            if resp is not None and resp.success:
                self.notif(
                    translate("UICoreRos", "Program paused."))
                return True

            else:

                self.notif(
                    translate("UICoreRos", "Failed to pause program."), temp=True, message_type=NotifyUserRequest.ERROR)
                return True

        else:

            rospy.logdebug("Attempt to pause/resume program in strange state: " +
                           str(self.state_manager.state.system_state))
            return False

    def cancel_cb(self):

        if self.state_manager.state.system_state in [
                InterfaceState.STATE_PROGRAM_RUNNING,
                InterfaceState.STATE_PROGRAM_STOPPED]:

            try:
                resp = self.program_stop_srv()
            except rospy.ServiceException:
                pass

            if resp is not None and resp.success:
                self.notif(
                    translate("UICoreRos", "Program stopped."))
                return True

            else:

                self.notif(
                    translate("UICoreRos", "Failed to stop program."), temp=True, message_type=NotifyUserRequest.ERROR)
                return True

        else:

            rospy.logdebug("Attempt to stop program in strange state: " + str(self.state_manager.state.system_state))
            return False

    def clear_all(self, include_dialogs=False):

        rospy.logdebug("Clear all")

        if self.current_instruction:
            items_to_keep = self.current_instruction.cleanup()

            if items_to_keep:

                rospy.loginfo(str(len(items_to_keep)) + " scene item(s) to be deleted later.")
                self.items_to_keep.extend(items_to_keep)

            self.current_instruction = None

        for vi in self.vis_instructions:
            vi.cleanup()

        self.vis_instructions = []

        super(UICoreRos, self).clear_all()

        if include_dialogs:

            for it in [self.program_list, self.program_vis]:

                if it is None:
                    continue
                try:
                    self.program_widget_pos = it.get_pos()
                except AttributeError:
                    pass
                break

            for it in [
                    self.program_error_dialog,
                    self.program_vis,
                    self.program_list]:

                if it is None:
                    continue
                self.remove_scene_items_by_type(type(it))
                it = None

    def state_visualizing(self, old_state, state, flags, system_state_changed):
        """For HoloLens visualization.
        Called everytime when system has changed and it's state is InterfaceState.STATE_VISUALIZE."""

        if system_state_changed:

            self.last_edited_prog_id = state.program_id

            if not self.ph.load(self.art.load_program(state.program_id)):

                self.notif(
                    translate(
                        "UICoreRos",
                        "Failed to load program from database."),
                    message_type=NotifyUserRequest.ERROR)

                # TODO what to do?
                return

            self.show_program_vis(visualize=True)

        if state.block_id == 0 or state.program_current_item.id == 0:
            rospy.logerr("Invalid state!")
            return

        # if visualize button in block view was hit
        if self.visualizing:

            # TODO how to avoid this?
            for ins in self.vis_instructions:
                if ins.block_id == state.block_id and ins.instruction_id == state.program_current_item.id:
                    self.program_vis.instruction = ins
                    break

            # select currently visualized instruction
            self.program_vis.set_active(
                state.block_id, state.program_current_item.id)

        # check flags for UI control
        for key, value in flags.items():
            if key == "HOLOLENS_VISUALIZATION":
                # if program has finished or user has stopped it with voice
                if value == "STOP":
                    self.program_vis.vis_stop_btn_cb(None)
                if value == "REPLAY":
                    self.program_vis.vis_replay_btn_cb(None)
                if value == "PAUSE":
                    self.program_vis.vis_pause_btn_cb(None)

    def create_hololens_state_msg(self, hololens_state, visualization_state=None):

        msg = HololensState()
        msg.hololens_state = hololens_state

        if hololens_state == HololensState.STATE_VISUALIZING and visualization_state is not None:
            msg.visualization_state = visualization_state
        # visualization not running at all
        else:
            msg.visualization_state = HololensState.VISUALIZATION_DISABLED

        return msg

    def v_visualize_cb(self):
        """Callback for VISUALIZE button in visualization mode.
            Notify HoloLens device that visualization started.
            Draw all elements of current program."""

        self.hololens_state_pub.publish(
            self.create_hololens_state_msg(
                HololensState.STATE_VISUALIZING,
                HololensState.VISUALIZATION_RUN))

        self.show_all_instructions_at_once(self.state_manager.state)

        self.visualizing = True

    def show_all_instructions_at_once(self, state):
        """Draws all drawable elements of program to table (like polygon from pick_from_polygon, place pose, etc.)"""
        block_id = state.block_id
        item_ids = self.ph.get_items_ids(block_id)

        for item_id in item_ids:
            self.show_instruction_visualization(block_id, item_id)

    def show_instruction_visualization(self, block_id, item_id):
        """Draws program visualization element based on block id and item id."""
        it = self.ph.get_item_msg(block_id, item_id)

        if self.ih[it.type].gui.vis:
            self.vis_instructions.append(self.ih[it.type].gui.vis(self, block_id, item_id))

    def v_back_cb(self):
        """Callback for BACK button in visualization mode.
            Notify HoloLens device that visualization ended."""

        self.hololens_state_pub.publish(self.create_hololens_state_msg(HololensState.STATE_IDLE))

        self.visualizing = False

        resp = None
        try:
            resp = self.stop_visualizing_srv()
        except rospy.ServiceException as e:
            print "Service call failed: %s" % e

        if resp is not None and resp.success:
            self.notif(
                translate("UICoreRos", "Program visualization stopped."))

        else:
            self.notif(
                translate(
                    "UICoreRos",
                    "Failed to stop program visualization."),
                temp=True,
                message_type=NotifyUserRequest.ERROR)
        return True

    def vis_pause_cb(self, visualization_paused):
        """Callback for PAUSE button while visualizing.
            Notify HoloLens device that pause/resume button was hit."""
        # if visualization is paused .. then resume it - e.g. hit RESUME button
        if visualization_paused:
            self.hololens_state_pub.publish(
                self.create_hololens_state_msg(
                    HololensState.STATE_VISUALIZING,
                    HololensState.VISUALIZATION_RESUME))
        # or visualization is running .. then pause it - e.g. hit PAUSE button
        else:
            self.hololens_state_pub.publish(
                self.create_hololens_state_msg(
                    HololensState.STATE_VISUALIZING,
                    HololensState.VISUALIZATION_PAUSE))

    def vis_stop_cb(self):
        """Callback for STOP button while visualizing.
            Notify HoloLens device that stop button was hit."""

        self.hololens_state_pub.publish(
            self.create_hololens_state_msg(
                HololensState.STATE_VISUALIZING,
                HololensState.VISUALIZATION_STOP))

        self.visualizing = False

    def vis_replay_cb(self):
        """Callback for REPLAY button while visualizing.
            Notify HoloLens device that replay button was hit."""

        self.hololens_state_pub.publish(
            self.create_hololens_state_msg(
                HololensState.STATE_VISUALIZING,
                HololensState.VISUALIZATION_REPLAY))

        self.visualizing = True

    def vis_back_to_blocks_cb(self):
        """Callback for BACK_TO_BLOCKS button while visualizing.
            Notify HoloLens device that visualization ended.
            Clear all drawed program visualization elements."""

        self.hololens_state_pub.publish(
            self.create_hololens_state_msg(
                HololensState.STATE_VISUALIZING,
                HololensState.VISUALIZATION_DISABLED))

        self.visualizing = False

        self.clear_all()

    def state_learning(self, old_state, state, flags, system_state_changed):

        if system_state_changed:

            self.last_edited_prog_id = state.program_id

            if not self.ph.load(self.art.load_program(state.program_id)):

                self.notif(
                    translate(
                        "UICoreRos",
                        "Failed to load program from database."),
                    message_type=NotifyUserRequest.ERROR)

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
            self.clear_all()

            self.learning_vis(state)

    def learning_vis(self, state):

        block_id = state.block_id
        item_id = state.program_current_item.id
        read_only = not state.edit_enabled

        if not self.ph.item_requires_learning(block_id, item_id):
            self.notif(translate("UICoreRos", "Item has no parameters."))
            return

        self.program_vis.editing_item = not read_only

        msg = self.ph.get_item_msg(block_id, item_id)

        if msg.type in self.ih.known_instructions():
            self.current_instruction = self.ih[msg.type].gui.learn(self, block_id, item_id,
                                                                   editable=state.edit_enabled)
        else:
            # TODO big error!
            rospy.logfatal("Unsupported instruction!")

        # TODO Edit/Done button not visible when there is work in progress!
        if block_id != self.program_vis.block_id or item_id != self.program_vis.item_id:
            self.program_vis.instruction = self.current_instruction  # TODO how to avoid this?
            self.program_vis.set_active(block_id, item_id)

        # TODO fix notified - how to get it from instruction?
        if read_only and self.current_instruction and not self.current_instruction.notified:

            if self.ph.item_has_nothing_to_set(block_id, item_id):
                # TODO check if it really uses reference
                self.notif(
                    translate("UICoreRos", "Instruction has nothing to set (uses reference)."))

            elif self.ph.item_learned(block_id, item_id):
                self.notif(
                    translate("UICoreRos", "Press 'Edit' to adjust selected instruction or 'Run' to test it out."))
            else:
                self.notif(
                    translate("UICoreRos", "Press 'Edit' to adjust selected instruction."))

    def active_item_switched(self, block_id, item_id, read_only=True, blocks=False):

        rospy.logdebug("Program ID:" + str(self.ph.get_program_id()) + ", active item ID: " +
                       str((block_id, item_id)) + ", blocks: " + str(blocks) + ", ro: " + str(read_only))

        if blocks:

            if self.ph.program_learned():
                self.notif(
                    translate("UICoreRos",
                              "All blocks are learned. Program may be saved using 'Done'"), temp=True)

            if block_id is None:
                self.notif(
                    translate(
                        "UICoreRos",
                        "Select program block and edit it. Press 'Done' to save changes and return to program list."))
            else:
                # get first program item from clicked block .. [1] because function
                # returns tuple - (block_id, item_id)
                # _item_id = self.ph.get_first_item_id(block_id=block_id)[1]
                # actualize InterfaceState msg with currently clicked block
                # if None not in (block_id, _item_id):
                #    self.state_manager.update_program_item(
                #        self.ph.get_program_id(), block_id, self.ph.get_item_msg(block_id, _item_id))

                if self.ph.block_learned(block_id):
                    self.notif(
                        translate("UICoreRos",
                                  "Block %1 is done. It can still be edited.").arg(block_id))

                else:

                    self.notif(
                        translate("UICoreRos",
                                  "Block %1 needs to be edited.").arg(block_id))

        else:

            if item_id is None:
                self.notif(
                    translate("UICoreRos",
                              "Select instruction or return to blocks."))

        if block_id and item_id is None:
            self.clear_all()

        if None not in (block_id, item_id):

            self.clear_all()  # TODO melo by se zavolat i pri odvybrani instrukce!

            self.state_manager.update_program_item(
                self.ph.get_program_id(), block_id, self.ph.get_item_msg(block_id, item_id))

            self.learning_vis(self.state_manager.state)

    def active_item_switched_for_visualization(self, block_id, item_id, read_only=True, blocks=False):
        """For HoloLens visualization. Called when clicked on specific block."""
        rospy.logdebug("Program ID:" + str(self.ph.get_program_id()) + ", active item ID: " +
                       str((block_id, item_id)) + ", blocks: " + str(blocks) + ", ro: " + str(read_only))

        # self.clear_all()

        if blocks:

            if block_id is None:
                self.notif(
                    translate("UICoreRos",
                              "Select program block and visualize it. Press 'Back' to return to program list."))
            else:

                if self.ph.block_learned(block_id):
                    self.notif(
                        translate("UICoreRos",
                                  "Press 'Visualize' for visualizing instructions of block %1.").arg(block_id))

                    # get first program item from clicked block .. [1] because function
                    # returns tuple - (block_id, item_id)
                    _item_id = self.ph.get_first_item_id(block_id=block_id)[1]
                    # actualize InterfaceState msg with currently clicked block
                    if None not in (block_id, _item_id):
                        self.state_manager.update_program_item(
                            self.ph.get_program_id(), block_id, self.ph.get_item_msg(block_id, _item_id))

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

        if not self.art.store_program(prog):

            self.notif(
                translate("UICoreRos", "Failed to store program"), temp=True, message_type=NotifyUserRequest.ERROR)
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

    def hololens_active_cb(self, msg):
        self.hololens_connected = msg.data

    def program_selected_cb(self, prog_id, run=False, template=False, visualize=False):

        self.template = template

        if run:

            (started, error) = self.art.start_program(prog_id)

            if not started:
                self.notif(translate("UICoreRos", "Failed to start program."), message_type=NotifyUserRequest.ERROR)
                rospy.logerr("Brain refused to start program: " + error)
                return

            self.notif(
                translate("UICoreRos", "Starting program %1...").arg(prog_id))
            self.program_list.set_enabled(False)

        # for hololens visualization
        elif visualize:
            if not self.ph.load(self.art.load_program(prog_id), template):

                self.notif(
                    translate(
                        "UICoreRos",
                        "Failed to load program from database."),
                    message_type=NotifyUserRequest.ERROR)
                return

            # TODO check if HoloLens are connected
            if self.hololens_connected:
                self.notif(
                    translate(
                        "UICoreRos",
                        "HoloLens device successfully contacted."),
                    message_type=NotifyUserRequest.INFO)
                req = ProgramIdTriggerRequest()
                req.program_id = self.ph.get_program_id()
                resp = None
                try:
                    resp = self.start_visualizing_srv(req)
                except rospy.ServiceException as e:
                    print "Service call failed: %s" % e

                if resp is None or not resp.success:
                    self.notif(
                        translate("UICoreRos", "Failed to start visualize mode."), message_type=NotifyUserRequest.ERROR)
            else:
                self.notif(
                    translate("UICoreRos", "Failed to contact HoloLens device."), message_type=NotifyUserRequest.ERROR)
                return

        else:

            if not self.ph.load(self.art.load_program(prog_id), template):

                self.notif(
                    translate(
                        "UICoreRos",
                        "Failed to load program from database."),
                    message_type=NotifyUserRequest.ERROR)

                # TODO what to do?
                return

            if template:

                # TODO this should be done by art_brain
                # if it is template - save it with new id
                headers = self.art.get_program_headers()
                prog = self.ph.get_program()
                prog.header.readonly = False
                ids = []

                for h in headers:
                    ids.append(h.id)

                # is there a better way how to find not used ID for program?
                for i in range(1, 2 ** 16 - 1):
                    if i not in ids:
                        prog.header.id = i
                        break
                else:
                    rospy.logerr("Failed to find available program ID")
                    return

                self.art.store_program(prog)

                rospy.loginfo("Program ID=" + str(prog_id) + " templated as ID=" + str(prog.header.id))

            req = ProgramIdTriggerRequest()
            req.program_id = self.ph.get_program_id()
            resp = None
            try:
                resp = self.start_learning_srv(req)
            except rospy.ServiceException as e:
                print "Service call failed: %s" % e

            if resp is None or not resp.success:

                self.notif(
                    translate("UICoreRos", "Failed to start edit mode."), message_type=NotifyUserRequest.ERROR)

            # else:
                # self.clear_all()
                # self.show_program_vis()

    def learning_request_cb(self, req):

        if req == LearningRequestGoal.GET_READY:
            self.notif(
                translate("UICoreRos", "Robot is getting ready for learning"))
            pass
        elif req == LearningRequestGoal.DONE:

            self.notif(translate("UICoreRos", "Robot is getting into default state"))

            self.current_instruction.learning_done()

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
            self.scene,
            self.program_widget_pos[0],
            self.program_widget_pos[1],
            headers_to_show,
            d,
            self.last_edited_prog_id,
            self.program_selected_cb,
            self.program_selection_changed_cb)

    def program_selection_changed_cb(self, program_id, ro=False, learned=False):

        if program_id is not None:

            if ro:

                if not learned:
                    self.notif(translate("UICoreRos", "Program is read-only and not leaned - it can be templated."))
                else:
                    self.notif(
                        translate(
                            "UICoreRos",
                            "Program is read-only and leaned - it can be templated or started."))

            else:

                if not learned:
                    self.notif(translate("UICoreRos", "Program needs to be learned. Use 'Edit' or 'Template'."))
                else:
                    self.notif(translate("UICoreRos", "Program learned - it is ready to be started."))

        else:

            self.notif(
                translate(
                    "UICoreRos",
                    "Please select a program. Use arrows to scroll the list. Tap program to select it."))

    def object_cb(self, msg):

        self.emit(QtCore.SIGNAL('objects'), msg)

    def object_cb_evt(self, msg):

        for obj_id in msg.lost_objects:

            self.remove_object(obj_id)

        for inst in msg.instances:

            obj = self.get_object(inst.object_id)

            if obj:
                obj.set_pos(inst.pose.position.x, inst.pose.position.y, inst.pose.position.z)
                obj.set_orientation(conversions.q2a(inst.pose.orientation))
            else:

                obj_type = self.art.get_object_type(inst.object_type)

                if obj_type:

                    self.add_object(
                        inst.object_id,
                        obj_type,
                        inst.pose.position.x,
                        inst.pose.position.y,
                        inst.pose.position.z,
                        conversions.q2a(
                            inst.pose.orientation),
                        self.object_selected)
                    # self.notif(translate("UICoreRos", "New object") + " ID=" + str(inst.object_id), temp=True)

                else:

                    rospy.logerr("Failed to get object type (" + inst.object_type + ") for ID=" + str(inst.object_id))

        if self.current_instruction:
            self.current_instruction.detected_objects(msg)

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

        self.current_instruction.object_selected(obj, selected, msg)

        self.state_manager.update_program_item(self.ph.get_program_id(
        ), self.program_vis.block_id, self.program_vis.get_current_item())
        return True
