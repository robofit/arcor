#!/usr/bin/env python
# coding=utf-8

import rospy
import sys
import importlib

import actionlib
from std_srvs.srv import Empty, EmptyRequest, Trigger, TriggerResponse
from art_msgs.srv import ProgramIdTrigger, ProgramIdTriggerResponse, \
    ObjectFlagClear, ObjectFlagSet
from std_msgs.msg import Bool
from art_msgs.msg import UserStatus, UserActivity, InterfaceState, SystemState, InstancesArray, \
    LearningRequestAction, LearningRequestGoal, LearningRequestResult
from shape_msgs.msg import SolidPrimitive
from art_msgs.srv import getObjectType, ProgramErrorResolveRequest, ProgramErrorResolveResponse, ProgramErrorResolve
import numpy as np
from art_helpers import InterfaceStateManager, ProgramHelper, ArtRobotHelper, \
    UnknownRobot, RobotParametersNotOnParameterServer, InstructionsHelper, InstructionsHelperException
from art_utils import ArtApiHelper

from tf import TransformListener
from art_brain import ArtBrainRobotInterface

from art_brain.brain_utils import ArtBrainUtils, ArtBrainErrors, ArtBrainErrorSeverities
from art_brain.art_brain_machine import ArtBrainMachine


# TODO:
# zjistovat jestli drzim objekt predtim nez zacnu neco vykonavat (typicky
# pick from feeder)  | DONE, otestovat

# při place zkontrolovat place pose (jestli tam není jiný objekt)


# update_state_manager - automaticky volat z machine

# polygon se neuloži hned ale (asi) až při ukončení učení/programu - při run to tam cpe starou hodnotu -
# v gui nebo v brainu - nenačte novou instrukci?

class ArtBrain(object):

    def __init__(self):

        self.ih = InstructionsHelper()
        states = []
        transitions = []
        self.instruction_fsm = {}
        for instruction in self.ih.known_instructions():
            states += self.ih[instruction].brain.fsm.states
            transitions += self.ih[instruction].brain.fsm.transitions
        self.fsm = ArtBrainMachine(states, transitions)

        for instruction in self.ih.known_instructions():
            self.instruction_fsm[instruction] = self.ih[instruction].brain.fsm(self)

        for _, fsm in self.instruction_fsm.iteritems():

            for state_function in fsm.state_functions:
                setattr(self.fsm, state_function, getattr(fsm, state_function))
        self.ih = None

        self.fsm.check_robot_in = self.check_robot_in
        self.fsm.check_robot_out = self.check_robot_out
        self.fsm.is_everything_calibrated = self.is_everything_calibrated
        self.fsm.state_init_ros = self.state_init_ros
        self.fsm.state_waiting_for_action = self.state_waiting_for_action
        self.fsm.state_program_init = self.state_program_init
        self.fsm.state_program_run = self.state_program_run
        self.fsm.state_learning_init = self.state_learning_init
        self.fsm.state_learning_run = self.state_learning_run
        self.fsm.learning_load_block_id = self.learning_load_block_id
        self.fsm.state_program_error = self.state_program_error
        self.fsm.state_program_paused = self.state_program_paused
        self.fsm.state_program_finished = self.state_program_finished
        self.fsm.state_program_load_instruction = self.state_program_load_instruction
        self.fsm.state_learning_step_done = self.state_learning_step_done
        self.fsm.state_learning_step_error = self.state_learning_step_error
        self.fsm.state_learning_done = self.state_learning_done
        self.fsm.state_update_program_item = self.state_update_program_item

        self.block_id = None
        self.user_id = 0
        self.objects = InstancesArray()
        self.executing_program = False
        self.program_paused = False
        self.program_pause_request = False
        self.learning = False
        self.instruction = None  # type: ProgramItem
        self.holding_object_left = None
        self.holding_object_right = None
        self.stop_server = False
        self.recalibrate = False
        self.table_calibrated = False
        self.table_calibrating = False
        self.cells_calibrated = False
        self.system_calibrated = False
        self.initialized = False
        self.projectors_calibrated = False
        self.last_drill_arm_id = None

        self.learning_block_id = None
        self.learning_item_id = None

        self.quit = False

        self.user_activity = None

        rospy.loginfo('Waiting for other nodes to come up...')

        self.program_resume_after_restart = rospy.get_param(
            "executing_program", False)
        self.learning_resume_after_restart = rospy.get_param(
            "learning_program", False)
        self.rh = None

        while not self.rh and not rospy.is_shutdown():
            try:
                self.rh = ArtRobotHelper()
            except UnknownRobot:
                ArtBrain.fatal("Unknown robot")
                return
            except RobotParametersNotOnParameterServer:
                rospy.logerr("Robot parameters not on parameter server yet...")
                rospy.sleep(1)

        try:
            p, m = rospy.get_param("robot_interface").rsplit('.', 1)
        except (KeyError, ValueError) as e:
            ArtBrain.fatal("Robot interface not set!")
            return
        try:
            mod = importlib.import_module(p)
            self.robot = getattr(mod, m)(self.rh)
        except (ImportError, AttributeError, TypeError) as e:
            ArtBrain.fatal("Failed to import robot interface: " + str(e))
            return

        if not isinstance(self.robot, ArtBrainRobotInterface):
            ArtBrain.fatal("Invalid robot interface.")
            return

        rospy.loginfo("Robot initialized")
        self.user_status_sub = rospy.Subscriber(
            "/art/user/status", UserStatus, self.user_status_cb)
        self.user_activity_sub = rospy.Subscriber(
            "/art/user/activity", UserActivity, self.user_activity_cb)
        self.table_calibrated_sub = rospy.Subscriber(
            "/art/interface/touchtable/calibrated", Bool, self.table_calibrated_cb)
        self.table_calibrating_sub = rospy.Subscriber(
            "/art/interface/touchtable/calibrating", Bool, self.table_calibrating_cb)
        self.system_calibrated_sub = rospy.Subscriber(
            "/art/system/calibrated", Bool, self.system_calibrated_cb)

        self.projectors_calibrated_sub = rospy.Subscriber(
            "/art/interface/projected_gui/app/projectors_calibrated", Bool, self.projectors_calibrated_cb)

        self.srv_program_start = rospy.Service(
            'program/start', ProgramIdTrigger, self.program_start_cb)
        self.srv_program_stop = rospy.Service(
            'program/stop', Trigger, self.program_stop_cb)

        self.srv_program_pause = rospy.Service(
            'program/pause', Trigger, self.program_pause_cb)
        self.srv_program_resume = rospy.Service(
            'program/resume', Trigger, self.program_resume_cb)

        self.srv_learning_start = rospy.Service(
            'learning/start', ProgramIdTrigger, self.learning_start_cb)
        self.srv_learning_stop = rospy.Service(
            'learning/stop', Trigger, self.learning_stop_cb)

        self.srv_learning_start = rospy.Service(
            '/art/brain/visualize/start', ProgramIdTrigger, self.visualize_start_cb)
        self.srv_learning_stop = rospy.Service(
            '/art/brain/visualize/stop', Trigger, self.visualize_stop_cb)

        self.srv_program_error_response = rospy.Service(
            'program/error_response',
            ProgramErrorResolve,
            self.program_error_response_cb)

        self.as_learning_request = actionlib.SimpleActionServer(
            "learning_request",
            LearningRequestAction,
            execute_cb=self.learning_request_cb,
            auto_start=True)

        self.state_manager = InterfaceStateManager(
            InterfaceState.BRAIN_ID,
            cb=self.interface_state_manager_cb)  # TODO callback?
        self.state_manager.set_system_state(
            InterfaceState.STATE_INITIALIZING)

        self.art = ArtApiHelper(brain=True)
        self.ph = ProgramHelper()

        self.objects_sub = rospy.Subscriber(
            "/art/object_detector/object_filtered",
            InstancesArray,
            self.objects_cb)

        # TODO use this topic instead of system_state in InterfaceState (duplication) ??
        # TODO move (pub/sub) to InterfaceStateManager?
        self.state_publisher = rospy.Publisher(
            "system_state", SystemState, queue_size=1)

        self.tf_listener = TransformListener()
        self.art.wait_for_api()

        self.get_obj_type_srv_client = ArtBrainUtils.create_service_client(
            '/art/db/object_type/get', getObjectType)
        # self.select_arm_srv_client = ArtBrainUtils.create_service_client(
        #    '/art/fuzzy/select_arm', SelectArm)
        self.clear_all_object_flags_srv_client = ArtBrainUtils.create_service_client(
            '/art/object_detector/flag/clear_all', Empty)
        self.clear_object_flag_srv_client = ArtBrainUtils.create_service_client(
            '/art/object_detector/flag/clear', ObjectFlagClear)
        self.set_object_flag_srv_client = ArtBrainUtils.create_service_client(
            '/art/object_detector/flag/set', ObjectFlagSet)

        # TODO 'hack' for experiment
        self.forearm_enable_srv_client = ArtBrainUtils.create_service_client(
            '/art/object_detector/forearm/enable', Empty)
        self.forearm_disable_srv_client = ArtBrainUtils.create_service_client(
            '/art/object_detector/forearm/disable', Empty)

        self.forearm_disable_srv_client.call()

        r = rospy.Rate(1)
        if not self.system_calibrated or not self.projectors_calibrated:
            self.robot.prepare_for_calibration()
        while not self.system_calibrated:
            if rospy.is_shutdown():
                return
            rospy.loginfo("Waiting for system calibration")
            r.sleep()

        self.calibrate_projectors_srv_client = ArtBrainUtils.create_service_client(
            '/art/interface/projected_gui/calibrate_projectors', Trigger)

        if not self.projectors_calibrated:

            resp = self.calibrate_projectors_srv_client.call()

            if not resp.success:

                rospy.logerr(
                    "Failed to start projector calibration: " +
                    resp.message)
                # TODO what to do?

            rospy.loginfo("Waiting for projectors to calibrate...")
            while not self.projectors_calibrated:
                if rospy.is_shutdown():
                    return
                rospy.sleep(1)

        if not self.table_calibrated:
            rospy.loginfo(
                'Waiting for /art/interface/touchtable/calibrate service')
            rospy.wait_for_service('/art/interface/touchtable/calibrate')
            rospy.loginfo(
                'Get /art/interface/touchtable/calibrate service')
            self.calibrate_table_srv_client = ArtBrainUtils.create_service_client(
                '/art/interface/touchtable/calibrate', Empty)

            attempt = 1
            rospy.sleep(1)
            while not self.table_calibrated:
                rospy.loginfo(
                    "Trying to calibrate table, attempt " + str(attempt))
                self.calibrate_table_srv_client.call()
                rospy.sleep(1)
                attempt += 1
                while not self.table_calibrated and self.table_calibrating:
                    rospy.sleep(1)
        r = rospy.Rate(1)
        while self.robot.halted and not rospy.is_shutdown():
            rospy.logwarn("Robot halted! Please unhalt!")
            r.sleep()

        self.try_robot_arms_get_ready()
        rospy.loginfo("Brain init done")
        self.initialized = True
        self.fsm.init()

    @staticmethod
    def fatal(msg):

        rospy.logfatal(msg)
        rospy.signal_shutdown(msg)

    # ***************************************************************************************
    #                                       STATES
    # ***************************************************************************************

    def state_init_ros(self, event):
        rospy.logdebug('Current state: state_init_ros')
        self.fsm.init_done()

    def state_waiting_for_action(self, event):
        rospy.logdebug('Current state: state_waiting_for_action')

        if self.program_resume_after_restart or self.learning_resume_after_restart:
            program_id = rospy.get_param("program_id", None)
            self.block_id = rospy.get_param("block_id", None)
            item_id = rospy.get_param("item_id", None)
            if self.block_id is None or item_id is None or program_id is None:
                rospy.logwarn("Could not resume program!")
                self.learning_resume_after_restart = False
                self.program_resume_after_restart = False
                return
            program = self.art.load_program(program_id)

            if not self.ph.load(program):
                rospy.logwarn("Could not resume program!")
                rospy.delete_param('program_id')
                rospy.delete_param('block_id')
                rospy.delete_param('item_id')
                self.state_manager.set_system_state(InterfaceState.STATE_IDLE)
                return
            if self.program_resume_after_restart:
                rospy.logdebug('Starting program')

                rospy.Timer(rospy.Duration(
                    1), self.program_start_timer_cb, oneshot=True)
                rospy.logdebug("program started")
            elif self.learning_resume_after_restart:
                self.learning_resume_after_restart = False
                rospy.logdebug('Starting learning')
                self.state_manager.update_program_item(program_id, self.block_id,
                                                       self.ph.get_item_msg(self.block_id, item_id), auto_send=True)
                self.state_manager.set_system_state(
                    InterfaceState.STATE_LEARNING)
                self.fsm.learning_start()
        else:
            self.state_manager.set_system_state(InterfaceState.STATE_IDLE)

    def state_shutdown(self, event):
        rospy.logdebug('Current state: state_shutdown')
        sys.exit()

    # ***************************************************************************************
    #                                  STATES PROGRAM
    # ***************************************************************************************

    def state_program_init(self, event):
        rospy.logdebug('Current state: state_program_init')
        rospy.logdebug('New program ready!')
        rospy.set_param("executing_program", True)

        if self.ph is None:
            self.fsm.error(severity=ArtBrainErrorSeverities.SEVERE,
                           error=ArtBrainErrors.ERROR_NO_PROGRAM_HELPER)
            return
        item_id = None
        if self.program_resume_after_restart:
            item_id = rospy.get_param("item_id", None)
        else:
            self.clear_all_object_flags_srv_client.call(EmptyRequest())

            (self.block_id, item_id) = self.ph.get_first_item_id()
        self.instruction = self.ph.get_item_msg(self.block_id, item_id)

        rospy.set_param("program_id", self.ph.get_program_id())
        rospy.set_param("block_id", self.block_id)
        rospy.set_param("item_id", item_id)
        self.executing_program = True
        self.program_paused = False
        self.program_pause_request = False
        if self.program_resume_after_restart:
            self.robot.init_arms(reset_holding_object=False)
        else:
            self.robot.init_arms(reset_holding_object=True)
        self.state_manager.set_system_state(
            InterfaceState.STATE_PROGRAM_RUNNING, auto_send=False)
        self.fsm.program_init_done()

    def state_program_run(self, event):
        rospy.logdebug('Current state: state_program_run')

        if not self.executing_program:
            self.fsm.finished()
            return
        if self.instruction is None:
            self.fsm.error(severity=ArtBrainErrorSeverities.SEVERE,
                           error=ArtBrainErrors.ERROR_NO_INSTRUCTION)
            return
        if not self.check_robot():
            return
        rospy.logdebug('Program id: ' +
                       str(self.ph.get_program_id()) +
                       ', item id: ' +
                       str(self.instruction.id) +
                       ', item type: ' +
                       str(self.instruction.type))

        for flag in self.instruction.flags:
            if flag.key == "CLEAR_OBJECT_FLAGS" and flag.value == "true":
                self.clear_all_object_flags_srv_client.call(EmptyRequest())

        '''while self.program_resume_after_restart and self.instruction.type in [ProgramItem.PLACE_TO_POSE,
                                                                           ProgramItem.PLACE_TO_GRID]:

            (self.block_id, item_id) = self.ph.get_id_on_success(
                self.block_id, self.instruction.id)

            if self.block_id == 0:
                self.fsm.finished()
                return

            self.instruction = self.ph.get_item_msg(self.block_id, item_id)
        '''
        self.program_resume_after_restart = False

        rospy.set_param("program_id", self.ph.get_program_id())
        rospy.set_param("block_id", self.block_id)
        rospy.set_param("item_id", self.instruction.id)
        try:
            self.instruction_fsm[self.instruction.type].run()
        except InstructionsHelperException:
            self.fsm.error()
            return

    def state_program_load_instruction(self, event):
        rospy.logdebug('Current state: state_program_load_instruction')
        success = event.kwargs.get('success', True)
        self.state_manager.set_error(0, 0)
        if not self.executing_program:
            self.fsm.finished()
            return
        if success:
            (self.block_id, item_id) = self.ph.get_id_on_success(
                self.block_id, self.instruction.id)
        else:
            (self.block_id, item_id) = self.ph.get_id_on_failure(
                self.block_id, self.instruction.id)

        if self.block_id == 0:
            self.fsm.finished()
            return

        self.instruction = self.ph.get_item_msg(self.block_id, item_id)

        if self.instruction is None:
            self.executing_program = False
            self.fsm.error()
            return
        if self.program_pause_request:
            self.fsm.pause()
            return
        self.fsm.done()

    def state_update_program_item(self, event):
        rospy.logerr("state_update_program_item")
        self.state_manager.update_program_item(
            self.ph.get_program_id(), self.block_id, self.instruction, auto_send=False)

    def state_program_paused(self, event):
        rospy.logdebug('Current state: state_program_paused')
        self.state_manager.set_system_state(
            InterfaceState.STATE_PROGRAM_STOPPED)
        self.state_manager.send()
        self.program_paused = True
        self.program_pause_request = False

    def state_program_finished(self, event):
        rospy.logdebug('Current state: state_program_finished')
        self.state_manager.set_system_state(
            InterfaceState.STATE_PROGRAM_FINISHED)
        self.robot.arms_reinit()
        self.state_manager.send()
        self.executing_program = False
        self.robot.arms_reinit()
        rospy.set_param("executing_program", False)
        rospy.delete_param("program_id")
        rospy.delete_param("block_id")
        rospy.delete_param("item_id")
        self.program_resume_after_restart = False
        self.fsm.done()

    def state_program_error(self, event):
        rospy.logdebug('Current state: state_program_error')
        severity = event.kwargs.get('severity', ArtBrainErrorSeverities.SEVERE)
        error = event.kwargs.get('error', None)
        rospy.logerr("Error: " + str(error))
        rospy.logerr("Severity of error: " + str(repr(severity)))

        if severity is None or error is None:
            severity = ArtBrainErrorSeverities.SEVERE
            error = ArtBrainErrors.ERROR_UNKNOWN
        self.state_manager.set_error(severity, error)

        if severity == ArtBrainErrorSeverities.SEVERE:
            # handle
            self.fsm.program_error_shutdown()
            return

        elif severity == ArtBrainErrorSeverities.ERROR:
            self.fsm.program_error_fatal()
            return

        elif severity == ArtBrainErrorSeverities.WARNING:
            if error == ArtBrainErrors.ERROR_OBJECT_MISSING or \
               error == ArtBrainErrors.ERROR_OBJECT_MISSING_IN_POLYGON:
                rospy.logwarn("Object is missing")
            elif error == ArtBrainErrors.ERROR_PICK_FAILED:
                rospy.logwarn("Pick failed")
                self.try_robot_arms_get_ready()
                self.robot.init_arms()

            rospy.logwarn("Waiting for user response")

            return

        elif severity == ArtBrainErrorSeverities.INFO:
            (self.block_id, item_id) = self.ph.get_id_on_failure(
                self.block_id, self.instruction.id)
            self.fsm.done(success=False)
        else:
            pass

        self.executing_program = False
        self.program_error_handled()

    # ***************************************************************************************
    #                                  STATES TEACHING
    # ***************************************************************************************

    def state_learning_init(self, event):
        rospy.logdebug('Current state: Teaching init')
        self.learning = True
        rospy.set_param("learning_program", True)
        self.fsm.init_done()

    def learning_load_block_id(self, event):
        self.block_id = self.state_manager.state.block_id

    def state_learning_run(self, event):
        rospy.logdebug('Current state: state_learning_run')

    def state_learning_step_error(self, event):
        rospy.logdebug('Current state: state_learning_step_error')
        severity = event.kwargs.get('severity', ArtBrainErrorSeverities.SEVERE)
        error = event.kwargs.get('error', ArtBrainErrors.ERROR_UNKNOWN)
        rospy.logdebug("Severity of error: " + str(severity))
        rospy.logdebug("Severity of error: " + str(error))
        self.state_manager.set_error(severity, error)
        self.state_manager.set_error(0, 0)

        if error is None:
            pass
            # TODO: kill brain
        if severity == ArtBrainErrorSeverities.SEVERE:
            pass
            # TODO: stop learning
        elif severity == ArtBrainErrorSeverities.ERROR:
            pass
        elif severity == ArtBrainErrorSeverities.WARNING:
            if error == ArtBrainErrors.ERROR_OBJECT_MISSING or \
               error == ArtBrainErrors.ERROR_OBJECT_MISSING_IN_POLYGON:
                rospy.logwarn("Object is missing")
            elif error == ArtBrainErrors.ERROR_PICK_FAILED:
                rospy.logwarn("Pick failed")
                self.try_robot_arms_get_ready()
                self.robot.init_arms()

        elif severity == ArtBrainErrorSeverities.INFO:

            pass
        self.fsm.error_handled()

    def state_learning_step_done(self, event):
        rospy.logdebug('Current state: state_learning_step_done')
        self.fsm.done()
        pass

    def state_learning_done(self, event):
        rospy.logdebug('Current state: state_learning_done')
        self.fsm.done()
        pass

    # ***************************************************************************************
    #                                     MANIPULATION
    # ***************************************************************************************

    def try_robot_arms_get_ready(self, arm_ids=[], max_attempts=3):
        assert isinstance(arm_ids, list)
        if self.robot.halted:
            return False

        attempt = 0
        while True:
            if attempt >= max_attempts:
                rospy.logerr("Failed to get ready")
                return False
            severity, error, arm_id = self.robot.arms_get_ready(arm_ids)
            if error is not None:
                attempt += 1
                rospy.logwarn(
                    "Error while getting ready: " +
                    str(arm_id) +
                    " , attempt: " +
                    str(attempt))
                continue
            else:
                rospy.loginfo("Robot ready")
                return True

    # ***************************************************************************************
    #                                        OTHERS
    # ***************************************************************************************

    def check_robot_in(self, event):
        self.check_robot()

    def check_robot_out(self, event):
        halted = event.kwargs.get('halted', False)
        print halted
        if halted:
            return
        self.check_robot()

    def program_start_timer_cb(self, event):
        self.fsm.program_start()

    def program_resume_timer_cb(self, event):
        self.state_manager.set_system_state(
            InterfaceState.STATE_PROGRAM_RUNNING)
        self.fsm.resume()

    def program_try_again_timer_cb(self, event):
        self.fsm.try_again()

    def program_skip_instruction_timer_cb(self, event):
        self.fsm.skip(success=True)

    def program_fail_instruction_timer_cb(self, event):
        self.fsm.fail(success=False)

    def update_state_manager(self):
        self.state_manager.update_program_item(
            self.ph.get_program_id(), self.block_id, self.instruction)

    def is_table_calibrated(self):
        return self.table_calibrated

    def is_everything_calibrated(self, event=None):

        return self.table_calibrated and self.system_calibrated

    def check_robot(self):
        if self.robot.is_halted():
            self.fsm.error(severity=ArtBrainErrorSeverities.WARNING,
                           error=ArtBrainErrors.ERROR_ROBOT_HALTED)
            return False
        else:
            return True

    def check_place_pose(self, place_pose, obj):
        w1 = self.get_object_max_width(obj)
        if w1 is None:
            return False
        for o in self.objects.instances:
            if o.object_id == obj.object_id:
                continue
            w2 = self.get_object_max_width(o)
            if w2 is None:
                # TODO: how to deal with this
                return False

            d = ArtBrainUtils.distance_2d(place_pose.pose, o.pose)
            if d < (w1 + w2):
                rospy.logerr('Another object too close to desired place pose')
                return False
        return True

    def check_gripper(self, gripper):
        if gripper is None:
            rospy.logerr("No gripper!")
            self.fsm.error(severity=ArtBrainErrorSeverities.SEVERE,
                           error=ArtBrainErrors.ERROR_NO_GRIPPER_AVAILABLE)
            return False

        if gripper.pp_client is None:
            rospy.logerr("No pick place client!")
            self.fsm.error(severity=ArtBrainErrorSeverities.SEVERE,
                           error=ArtBrainErrors.ERROR_GRIPPER_PP_CLIENT_MISSING)
            return False

        if not gripper.pp_client.wait_for_server(rospy.Duration(2)):
            rospy.logerr("Pick place server is not running!")
            self.fsm.error(severity=ArtBrainErrorSeverities.WARNING,
                           error=ArtBrainErrors.ERROR_PICK_PLACE_SERVER_NOT_READY)
            return False

        return True

    def check_gripper_for_place(self, gripper):
        if not self.check_gripper(gripper):
            return False

        if gripper.holding_object is None:
            rospy.logwarn("Place: gripper " + gripper.name +
                          " is not holding object")
            self.fsm.error(severity=ArtBrainErrorSeverities.WARNING,
                           error=ArtBrainErrors.ERROR_NO_OBJECT_IN_GRIPPER)
            return False

        return True

    def check_gripper_for_pick(self, gripper):
        if not self.check_gripper(gripper):
            return False
        if gripper.holding_object is not None:
            rospy.logwarn(
                "Pick: gripper " +
                gripper.name +
                " already holding an object (" +
                gripper.holding_object.object_id +
                ")")
            self.fsm.error(severity=ArtBrainErrorSeverities.WARNING,
                           error=ArtBrainErrors.ERROR_OBJECT_IN_GRIPPER)
            return False

        return True

    # ***************************************************************************************
    #                                     ROS COMMUNICATION
    # ***************************************************************************************

    def program_pause_cb(self, req):
        resp = TriggerResponse()
        resp.success = True
        if not self.executing_program:
            resp.success = False
            resp.message = "Program si not running!"
        else:
            self.program_pause_request = True
        return resp

    def program_resume_cb(self, req):
        resp = TriggerResponse()
        resp.success = True
        if not self.executing_program:
            resp.success = False
            resp.message = "Program si not running!"
        elif not self.program_paused:
            resp.success = False
            resp.message = "Program is not paused"
        else:
            self.program_paused = False
            rospy.Timer(rospy.Duration(
                1), self.program_resume_timer_cb, oneshot=True)
        return resp

    def program_start_cb(self, req):

        resp = ProgramIdTriggerResponse()

        if not self.is_everything_calibrated():
            resp.success = False
            resp.error = 'Something is not calibrated'
            rospy.logwarn('Something is not calibrated')
            return resp

        if self.executing_program:

            resp.success = False
            resp.error = 'Program already running'
            return resp

        if not self.fsm.is_waiting_for_action():
            resp.success = False
            resp.error = 'Not ready for program start!'
            return resp

        rospy.logdebug('Loading program ' +
                       str(req.program_id) + ' from db...')

        program = self.art.load_program(req.program_id)

        if not self.ph.load(program):
            resp.success = False
            resp.error = 'Cannot get program'
            return resp

        rospy.logdebug('Starting program')

        rospy.Timer(rospy.Duration(
            1), self.program_start_timer_cb, oneshot=True)
        rospy.logdebug("program started")
        resp.success = True
        return resp

    def program_stop_cb(self, req):
        resp = TriggerResponse()
        resp.success = True
        if self.executing_program:
            rospy.logdebug('Stopping program')
            self.executing_program = False
            if self.program_paused:
                self.program_paused = False
                rospy.Timer(rospy.Duration(
                    0, 100), self.program_resume_timer_cb, oneshot=True)
        else:
            resp.success = False
            resp.message = "Program is not running"

        return resp

    def program_error_response_cb(self, req):
        resp = ProgramErrorResolveResponse()
        if not self.fsm.is_program_error():
            resp.success = False
            return resp
        rospy.logdebug('We\'ve got response to our program error')
        if req.user_response_type == ProgramErrorResolveRequest.TRY_AGAIN:
            self.state_manager.set_error(0, 0)
            rospy.Timer(rospy.Duration(
                1), self.program_try_again_timer_cb, oneshot=True)
        elif req.user_response_type == ProgramErrorResolveRequest.SKIP_INSTRUCTION:
            rospy.Timer(rospy.Duration(
                1), self.program_skip_instruction_timer_cb, oneshot=True)
        elif req.user_response_type == ProgramErrorResolveRequest.FAIL_INSTRUCTION:
            rospy.Timer(rospy.Duration(
                1), self.program_fail_instruction_timer_cb, oneshot=True)
        elif req.user_response_type == ProgramErrorResolveRequest.END_PROGRAM:
            self.fsm.program_error_fatal()
        resp.success = True
        return resp

    def learning_start_cb(self, req):
        resp = ProgramIdTriggerResponse()
        resp.success = False

        if not self.is_everything_calibrated():

            resp.error = 'Something is not calibrated'
            rospy.logwarn('Something is not calibrated')
            return resp

        if not self.fsm.is_waiting_for_action():

            resp.error = 'Not ready for learning start!'
            rospy.logwarn('Not ready for learning start!')
            return resp

        program = self.art.load_program(req.program_id)

        if not self.ph.load(program):
            resp.success = False
            resp.error = 'Cannot get program.'
            return resp

        rospy.logdebug('Starting learning')
        (self.block_id, item_id) = self.ph.get_first_item_id()
        self.state_manager.update_program_item(
            req.program_id, self.block_id, self.ph.get_item_msg(
                self.block_id, item_id), auto_send=False)
        self.state_manager.set_system_state(InterfaceState.STATE_LEARNING)
        resp.success = True
        self.fsm.learning_start()
        return resp

    def visualize_start_cb(self, req):
        resp = ProgramIdTriggerResponse()
        resp.success = False

        if not self.is_everything_calibrated():

            resp.error = 'Something is not calibrated'
            rospy.logwarn('Something is not calibrated')
            return resp

        if not self.fsm.is_waiting_for_action():

            resp.error = 'Not ready for visualize start!'
            rospy.logwarn('Not ready for visualize start!')
            return resp

        program = self.art.load_program(req.program_id)

        if not self.ph.load(program):
            resp.success = False
            resp.error = 'Cannot get program.'
            return resp

        rospy.logdebug('Starting visualize')
        (self.block_id, item_id) = self.ph.get_first_item_id()
        self.state_manager.update_program_item(
            req.program_id, self.block_id, self.ph.get_item_msg(
                self.block_id, item_id), auto_send=False)
        self.state_manager.set_system_state(InterfaceState.STATE_VISUALIZE)
        resp.success = True
        self.fsm.visualize_start()
        return resp

    def learning_stop_cb(self, req):
        resp = TriggerResponse()
        if not self.fsm.is_learning_run:
            resp.success = False
        rospy.logdebug('Stopping learning')
        self.learning = False
        rospy.set_param("learning_program", False)
        resp.success = True
        self.fsm.learning_done()
        return resp

    def visualize_stop_cb(self, req):
        resp = TriggerResponse()
        if not self.fsm.is_visualize_run:
            resp.success = False
        rospy.logdebug('Stopping visualize')
        resp.success = True
        self.fsm.visualize_done()
        return resp

    def interface_state_manager_cb(self,
                                   state,  # type: InterfaceState
                                   msg,  # type: InterfaceState
                                   flags):
        if msg.interface_id != InterfaceState.BRAIN_ID:
            if msg.system_state == InterfaceState.STATE_LEARNING:
                self.ph.set_item_msg(msg.block_id, msg.program_current_item)
                rospy.set_param("program_id", self.ph.get_program_id())
                rospy.set_param("block_id", self.block_id)
                rospy.set_param("item_id", msg.program_current_item.id)
                self.art.store_program(self.ph.get_program())
        pass

    def projectors_calibrated_cb(self, msg):

        self.projectors_calibrated = msg.data

    def user_status_cb(self, req):
        self.user_id = req.user_id

    def user_activity_cb(self, req):
        self.user_activity = req.activity

    def objects_cb(self, req):
        self.objects = req

    def table_calibrated_cb(self, req):
        self.table_calibrated = req.data

    def table_calibrating_cb(self, req):
        self.table_calibrating = req.data

    def system_calibrated_cb(self, req):
        self.system_calibrated = req.data

    def get_object_max_width(self, obj):
        if obj is None:
            rospy.logerr('No object is specified')
            return None
        try:
            obj_type = self.get_obj_type_srv_client.call(obj.object_type)
            if not obj_type.success:
                rospy.logerr('No object with id ' +
                             str(obj.object_id) + ' found')
                return None
            if obj_type.object_type.bbox.type != SolidPrimitive.BOX:
                rospy.logerr(
                    'Sorry, only BOX type objects are supported at the moment')
            x = obj_type.object_type.bbox.dimensions[SolidPrimitive.BOX_X]
            y = obj_type.object_type.bbox.dimensions[SolidPrimitive.BOX_Y]
            return np.hypot(x / 2, y / 2)
        except rospy.ServiceException as e:
            rospy.logerr('Service call failed: ' + str(e))
            return None

    def learning_request_cb(self, goal):
        result = LearningRequestResult()
        if not self.fsm.is_learning_run:
            result.success = False
            result.message = "Not in learning mode!"
        rospy.logdebug("Learning_request goal: " + str(goal.request))

        instruction = self.state_manager.state.program_current_item  # type: ProgramItem
        self.instruction = instruction

        self.state_manager.state.edit_enabled = False
        self.state_manager.send()

        if goal.request == LearningRequestGoal.GET_READY:

            self.state_manager.state.edit_enabled = True

            if self.fsm.is_learning_run:
                self.instruction_fsm[instruction.type].learning()
                # TODO: really?
                result.success = True
                self.state_manager.state.edit_enabled = True
                self.state_manager.send()
                self.as_learning_request.set_succeeded(result)
            else:
                result.success = False
                result.message = "Not in learning state!"
                self.as_learning_request.set_aborted(result)

            # TODO: handle error
        elif goal.request == LearningRequestGoal.EXECUTE_ITEM:
            self.ph.set_item_msg(
                self.state_manager.state.block_id, instruction)

            # TODO let ui(s) know that item is being executed

            # self.fsm.error(severity=ArtBrainErrorSeverities.INFO,
            #                error=ArtBrainErrorSeverities.ERROR_LEARNING_NOT_IMPLEMENTED)
            if self.fsm.is_learning_run:
                self.instruction_fsm[instruction.type].learning_run()
                # TODO: really?
                result.success = True

                self.as_learning_request.set_succeeded(result)
            else:
                result.success = False
                result.message = "Not in learning state!"
                self.as_learning_request.set_aborted(result)

        elif goal.request == LearningRequestGoal.DONE:
            # Great!
            result.success = True

            self.fsm.done()
            self.as_learning_request.set_succeeded(result)
        else:

            result.success = False
            result.message = "Unkwnown request"

            self.as_learning_request.set_aborted(result)


if __name__ == '__main__':
    rospy.init_node('art_brain_node', log_level=rospy.DEBUG)

    try:
        node = ArtBrain()

        '''
        rate = rospy.Rate(100)

        while not rospy.is_shutdown():
            # node.process()
            rate.sleep()'''

        rospy.spin()
    except rospy.ROSInterruptException:
        pass
