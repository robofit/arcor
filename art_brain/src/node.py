#!/usr/bin/env python
# coding=utf-8

import rospy
import time
import copy
import sys

import actionlib
from art_msgs.msg import LocalizeAgainstUMFAction, LocalizeAgainstUMFGoal, LocalizeAgainstUMFResult
from std_srvs.srv import Empty, EmptyRequest, EmptyResponse, Trigger, TriggerResponse
from art_msgs.msg import UserStatus, UserActivity, InterfaceState
from art_msgs.srv import startProgram, startProgramResponse, getProgram
from geometry_msgs.msg import PoseStamped, Pose
from std_msgs.msg import String, Bool
from art_msgs.msg import PickPlaceAction, PickPlaceGoal, SystemState, ObjInstance, InstancesArray, ProgramItem, \
    ObjectType, LearningRequestAction, LearningRequestGoal, LearningRequestResult
from shape_msgs.msg import SolidPrimitive
from art_msgs.srv import getObjectType, ProgramErrorResolveRequest, ProgramErrorResolveResponse, ProgramErrorResolve
import matplotlib.path as mplPath
import numpy as np
import random
from art_utils import InterfaceStateManager, ArtApiHelper, ProgramHelper

from tf import TransformerROS, TransformListener


import logging
from transitions import logger

from art_brain.brain_utils import ArtBrainUtils, ArtGripper, ArtBrainErrors
from art_brain.art_brain_machine import ArtBrainMachine


# TODO:
# !!!!!!!!!!!!!!sudo pip install enum34

# pause/resume programu
# používat obě ramena robota -> done, otestovat na stole
# při place zkontrolovat place pose (jestli tam není jiný objekt) -> done, otestovat na stole

# action pro provedení naučené instrukce

# update_state_manager - automaticky volat z machine

class ArtBrain(object):

    def __init__(self):
        self.fsm = ArtBrainMachine()

        # map callbacks to fsm
        self.fsm.is_everything_calibrated = self.is_everything_calibrated
        self.fsm.state_init_ros = self.state_init_ros
        self.fsm.state_waiting_for_action = self.state_waiting_for_action
        self.fsm.state_program_init = self.state_program_init
        self.fsm.state_program_run = self.state_program_run
        self.fsm.state_get_ready = self.state_get_ready
        self.fsm.state_wait_for_user = self.state_wait_for_user
        self.fsm.state_wait_until_user_finishes = self.state_wait_until_user_finishes
        self.fsm.state_pick_from_polygon = self.state_pick_from_polygon
        self.fsm.state_pick_from_feeder = self.state_pick_from_feeder
        self.fsm.state_pick_object_id = self.state_pick_object_id
        self.fsm.state_place_to_pose = self.state_place_to_pose
        self.fsm.state_path_through_points = self.state_path_through_points
        self.fsm.state_welding_points = self.state_welding_points
        self.fsm.state_welding_seam = self.state_welding_seam
        self.fsm.state_drill_points = self.state_drill_points
        self.fsm.state_program_error = self.state_program_error
        self.fsm.state_program_paused = self.state_program_paused
        self.fsm.state_program_finished = self.state_program_finished
        self.fsm.state_program_load_instruction = self.state_program_load_instruction
        self.fsm.state_learning_init = self.state_learning_init
        self.fsm.state_learning_run = self.state_learning_run
        self.fsm.state_learning_pick_from_polygon = self.state_learning_pick_from_polygon
        self.fsm.state_learning_pick_from_feeder = self.state_learning_pick_from_feeder
        self.fsm.state_learning_pick_object_id = self.state_learning_pick_object_id
        self.fsm.state_learning_place_to_pose = self.state_learning_place_to_pose
        self.fsm.state_learning_pick_from_polygon_run = self.state_learning_pick_from_polygon_run
        self.fsm.state_learning_pick_from_feeder_run = self.state_learning_pick_from_feeder_run
        self.fsm.state_learning_pick_object_id_run = self.state_learning_pick_object_id_run
        self.fsm.state_learning_place_to_pose_run = self.state_learning_place_to_pose_run
        self.fsm.state_learning_wait = self.state_learning_wait
        self.fsm.state_learning_step_done = self.state_learning_step_done
        self.fsm.state_learning_step_error = self.state_learning_step_error
        self.fsm.state_learning_done = self.state_learning_done
        self.fsm.state_learning_pick_from_feeder_exit = self.state_learning_pick_from_feeder_exit
        self.fsm.state_shutdown = self.state_shutdown

        self.block_id = None
        self.user_id = 0
        self.objects = InstancesArray()
        self.executing_program = False
        self.program_paused = False
        self.program_pause_request = False
        self.learning = False
        self.instruction = None
        self.holding_object_left = None
        self.holding_object_right = None
        self.stop_server = False
        self.recalibrate = False
        self.table_calibrated = False
        self.table_calibrating = False
        self.cells_calibrated = False
        self.system_calibrated = False
        self.motors_halted = True
        self.initialized = False
        self.projectors_calibrated = False

        self.learning_block_id = None
        self.learning_item_id = None

        self.quit = False

        self.user_activity = None

        rospy.loginfo('Waiting for other nodes to come up...')

        self.show_marker_service = rospy.get_param(
            'show_marker_service', '/art/interface/projected_gui/show_marker')
        self.hide_marker_service = rospy.get_param(
            'hide_marker_service', '/art/interface/projected_gui/hide_marker')
        self.table_localize_action = rospy.get_param(
            'table_localize_action', '/umf_localizer_node_table/localize')
        self.pr2_localize_action = rospy.get_param(
            'pr2_localize_action', '/umf_localizer_node/localize')

        self.calibrate_pr2 = rospy.get_param('calibrate_pr2', False)
        self.calibrate_table = rospy.get_param('calibrate_table', False)

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
        self.motors_halted_sub = rospy.Subscriber(
            "/pr2_ethercat/motors_halted", Bool, self.motors_halted_cb)
        self.projectors_calibrated_sub = rospy.Subscriber(
            "/art/interface/projected_gui/app/projectors_calibrated", Bool, self.projectors_calibrated_cb)

        self.srv_program_start = rospy.Service(
            '/art/brain/program/start', startProgram, self.program_start_cb)
        self.srv_program_stop = rospy.Service(
            '/art/brain/program/stop', Trigger, self.program_stop_cb)

        self.srv_program_pause = rospy.Service(
            '/art/brain/program/pause', Trigger, self.program_pause_cb)
        self.srv_program_resume = rospy.Service(
            '/art/brain/program/resume', Trigger, self.program_resume_cb)

        self.srv_learning_start = rospy.Service(
            '/art/brain/learning/start', Trigger, self.learning_start_cb)
        self.srv_learning_stop = rospy.Service(
            '/art/brain/learning/stop', Trigger, self.learning_stop_cb)

        self.srv_program_error_response = rospy.Service(
            '/art/brain/program/error_response',
            ProgramErrorResolve,
            self.program_error_response_cb)

        self.as_learning_request = actionlib.SimpleActionServer(
            "/art/brain/learning_request",
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
            "/art/brain/system_state", SystemState, queue_size=1)

        self.tf_listener = TransformListener()

        self.gripper_usage = ArtGripper.GRIPPER_BOTH
        gripper = rospy.get_param('gripper_usage', 'both')
        self.left_gripper = None
        self.right_gripper = None
        if gripper == 'left':
            self.gripper_usage = ArtGripper.GRIPPER_LEFT
        elif gripper == 'right':
            self.gripper_usage = ArtGripper.GRIPPER_RIGHT
        elif gripper == 'both':
            self.gripper_usage = ArtGripper.GRIPPER_BOTH

        if self.gripper_usage == ArtGripper.GRIPPER_BOTH or ArtGripper.GRIPPER_LEFT:
            self.left_gripper = ArtGripper('left_arm')
        else:
            self.left_gripper = None
        if self.gripper_usage == ArtGripper.GRIPPER_BOTH or ArtGripper.GRIPPER_RIGHT:
            self.right_gripper = ArtGripper('right_arm')
        else:
            self.right_gripper = None

        self.art.wait_for_api()

        self.get_obj_type_srv_client = ArtBrainUtils.create_service_client(
            '/art/db/object_type/get', getObjectType)
        # self.select_arm_srv_client = ArtBrainUtils.create_service_client(
        #    '/art/fuzzy/select_arm', SelectArm)

        r = rospy.Rate(1)
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

                rospy.logerr("Failed to start projector calibration: " + resp.message)
                # TODO what to do?

            rospy.loginfo("Waiting for projectors to calibrate...")
            while not self.projectors_calibrated:

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

        self.initialized = True
        self.fsm.init()

    # ***************************************************************************************
    #                                       STATES
    # ***************************************************************************************

    def state_init_ros(self, event):
        rospy.loginfo("state_init_ros")
        self.fsm.init_done()

    def state_waiting_for_action(self, event):
        rospy.loginfo('state_waiting_for_action')
        self.state_manager.set_system_state(InterfaceState.STATE_IDLE)

    def state_shutdown(self, event):
        rospy.loginfo('state_shutdown')
        sys.exit()

    # ***************************************************************************************
    #                                  STATES PROGRAM
    # ***************************************************************************************

    def state_program_init(self, event):
        rospy.loginfo('state_program_init')
        rospy.loginfo('New program ready!')

        if self.ph is None:
            self.fsm.error(severity=InterfaceState.SEVERE,
                           error=ArtBrainErrors.ERROR_NO_PROGRAM_HELPER)
            return

        (self.block_id, item_id) = self.ph.get_first_item_id()
        self.instruction = self.ph.get_item_msg(self.block_id, item_id)

        self.executing_program = True
        self.program_paused = False
        self.program_pause_request = False
        if self.left_gripper is not None:
            self.left_gripper.re_init
        if self.right_gripper is not None:
            self.right_gripper.re_init
        self.state_manager.set_system_state(
            InterfaceState.STATE_PROGRAM_RUNNING, auto_send=False)
        self.fsm.program_init_done()

    def state_program_run(self, event):
        rospy.loginfo('state_program_run')

        if not self.executing_program:
            self.finished()
            return
        if self.instruction is None:
            self.fsm.error(severity=InterfaceState.SEVERE,
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

        instructions = {
            ProgramItem.GET_READY: self.fsm.get_ready,
            ProgramItem.PICK_FROM_POLYGON: self.fsm.pick_from_polygon,
            ProgramItem.PICK_FROM_FEEDER: self.fsm.pick_from_feeder,
            ProgramItem.PICK_OBJECT_ID: self.fsm.pick_object_id,
            ProgramItem.PLACE_TO_POSE: self.fsm.place_to_pose,
            ProgramItem.PATH_THROUGH_POINTS: self.fsm.path_through_points,
            ProgramItem.WELDING_POINTS: self.fsm.welding_points,
            ProgramItem.WELDING_SEAM: self.fsm.welding_seam,
            ProgramItem.DRILL_POINTS: self.fsm.drill_points,
            ProgramItem.WAIT_FOR_USER: self.fsm.wait_for_user,
            ProgramItem.WAIT_UNTIL_USER_FINISHES: self.fsm.wait_until_user_finishes,
        }

        instruction_transition = instructions.get(self.instruction.type, None)
        print (instruction_transition)
        print (self.instruction.type)
        if instruction_transition is None:
            self.fsm.error()
            return
        instruction_transition()

        '''if not self.executing_program:
            self.fsm.error(severity=InterfaceState.SEVERE,
                           error=ArtBrainErrors.ERROR_NOT_EXECUTING_PROGRAM)
            return'''

    def state_pick_from_polygon(self, event):
        rospy.loginfo('state_pick_from_polygon')
        if not self.check_robot():
            return
        self.pick_object_from_polygon(self.instruction)

    def state_pick_from_feeder(self, event):
        rospy.loginfo('state_pick_from_feeder')
        if not self.check_robot():
            return
        obj = ArtBrainUtils.get_pick_obj_from_feeder(self.instruction)
        if obj is None:
            self.fsm.error(severity=InterfaceState.ERROR,
                           error=ArtBrainErrors.ERROR_OBJECT_NOT_DEFINED)
            return
        '''if not self.ph.is_pick_pose_set(self.block_id, self.instruction.id):
            self.fsm.error(severity=InterfaceState.WARNING,
                           error=ArtBrainErrors.ERROR_PICK_POSE_NOT_SELECTED)
            return'''
        gripper = self.get_gripper(pick_pose=self.instruction.pose)
        if not self.check_gripper_for_pick(gripper):
            return

        # TODO: pick from feeder method
        if self.pick_object_from_feeder(obj, gripper, self.instruction.pose):
            gripper.holding_object = obj
            gripper.last_pick_instruction_id = self.instruction.id
            self.done(success=True)

        else:
            gripper.get_ready_clinet.call()
            self.done(success=False)

    def state_pick_object_id(self, event):
        if not self.check_robot():
            return
        rospy.loginfo('state_pick_object_id')
        obj = ArtBrainUtils.get_pick_obj(self.instruction, self.objects)
        if obj is None or obj.object_id is None:
            self.fsm.error(severity=InterfaceState.WARNING,
                           error=ArtBrainErrors.ERROR_OBJECT_MISSING)
            self.state_manager.update_program_item(
                self.ph.get_program_id(), self.block_id, self.instruction)
            return
        self.state_manager.update_program_item(
            self.ph.get_program_id(), self.block_id, self.instruction, {
                "SELECTED_OBJECT_ID": obj.object_id})
        gripper = self.get_gripper(obj=obj)
        if not self.check_gripper_for_pick(gripper):
            return

        if self.pick_object_by_id(obj, gripper):
            gripper.holding_object = obj
            gripper.last_pick_instruction_id = self.instruction.id
            self.fsm.done()

        else:
            gripper.get_ready_clinet.call()
            self.fsm.error(severity=InterfaceState.WARNING,
                           error=ArtBrainErrors.ERROR_PICK_FAILED)

    def state_place_to_pose(self, event):
        rospy.loginfo('state_place_to_pose')
        if not self.check_robot():
            return
        self.place_object_to_pose(self.instruction)

    def state_path_through_points(self, event):
        rospy.loginfo('state_path_through_points')
        if not self.check_robot():
            return
        gripper = self.get_gripper_path_following()
        if gripper.move_through_poses(self.instruction.pose):
            self.fsm.done()
        else:
            # TODO: error
            return

    def state_welding_points(self, event):
        rospy.loginfo('state_welding_points')
        if not self.check_robot():
            return
        gripper = self.get_gripper_path_following()  # TODO:
        if gripper.touch_poses(self.instruction.pose):
            self.fsm.done()
        else:
            # TODO: error
            return

    def state_welding_seam(self, event):
        rospy.loginfo('state_welding_seam')
        if not self.check_robot():
            return
        gripper = self.get_gripper_path_following()  # TODO:
        if gripper.move_through_poses(self.instruction.pose):
            self.fsm.done()
        else:
            # TODO: error
            return

    def state_drill_points(self, event):
        rospy.loginfo('state_drill_points')
        if not self.check_robot():
            return
        gripper = self.get_gripper_path_following()  # TODO:
        if gripper.touch_poses(self.instruction.pose, drill_duration=5):
            self.fsm.done()
        else:
            # TODO: error
            return

    def state_wait_for_user(self, event):
        rospy.loginfo('state_wait_for_user')

        self.state_manager.update_program_item(
            self.ph.get_program_id(), self.block_id, self.instruction)

        rate = rospy.Rate(10)

        while self.user_activity != UserActivity.READY:
            rate.sleep()

        self.fsm.done(success=True)

    def state_wait_until_user_finishes(self, event):
        rospy.loginfo('state_wait_until_user_finishes')

        self.state_manager.update_program_item(
            self.ph.get_program_id(), self.block_id, self.instruction)

        rate = rospy.Rate(10)

        while self.user_activity != UserActivity.WORKING:
            rate.sleep()

        self.fsm.done(success=True)

    def state_get_ready(self, event):
        rospy.loginfo('state_get_ready')
        if not self.check_robot():
            return
        self.state_manager.update_program_item(
            self.ph.get_program_id(), self.block_id, self.instruction)
        # TODO: call some service to set PR2 to ready position
        # TODO handle if it fails
        self.right_gripper.get_ready()
        self.left_gripper.get_ready()
        self.fsm.done(success=True)

    def state_program_load_instruction(self, event):
        rospy.loginfo('state_program_load_instruction')
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

    def state_program_paused(self, event):
        rospy.loginfo('state_program_paused')
        self.state_manager.set_system_state(
            InterfaceState.STATE_PROGRAM_STOPPED)
        self.state_manager.send()
        self.program_paused = True
        self.program_pause_request = False

    def state_program_finished(self, event):
        rospy.loginfo('state_program_finished')
        self.state_manager.set_system_state(
            InterfaceState.STATE_PROGRAM_FINISHED)
        self.state_manager.send()
        self.executing_program = False
        self.fsm.done()

    def state_program_error(self, event):
        rospy.loginfo('state_program_error')
        severity = event.kwargs.get('severity', InterfaceState.SEVERE)
        error = event.kwargs.get('error', None)

        if severity is None or error is None:
            severity = InterfaceState.SEVERE
            error = ArtBrainErrors.ERROR_UNKNOWN
        rospy.logerr("Error: " + str(error))
        self.state_manager.set_error(severity, error)
        self.state_manager.set_error(0, 0)
        if severity == InterfaceState.SEVERE:
            # handle
            self.fsm.program_error_shutdown()
            return

        elif severity == InterfaceState.ERROR:
            self.fsm.program_error_fatal()
            return

        elif severity == InterfaceState.WARNING:
            if error == ArtBrainErrors.ERROR_OBJECT_MISSING or \
               error == ArtBrainErrors.ERROR_OBJECT_MISSING_IN_POLYGON:
                rospy.logwarn("Object is missing")
            elif error == ArtBrainErrors.ERROR_PICK_FAILED:
                rospy.logwarn("Pick failed")
                self.left_gripper.get_ready()
                self.left_gripper.re_init()
                self.right_gripper.get_ready()
                self.right_gripper.re_init()
            rospy.logwarn("Waiting for user response")

            return

        elif severity == InterfaceState.INFO:
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
        rospy.loginfo('Teaching init')
        self.learning = True
        self.fsm.init_done()

    def state_learning_run(self, event):
        rospy.loginfo('state_learning_run')
        pass

    def state_learning_pick_from_polygon(self, event):
        rospy.loginfo('state_learning_pick_from_polygon')
        # i have nothing to do yet

        pass

    def state_learning_pick_from_polygon_run(self, event):
        rospy.loginfo('state_learning_pick_from_polygon_run')
        instruction = self.state_manager.state.program_current_item  # type: ProgramItem
        self.pick_object_from_polygon(instruction, update_state_manager=False)
        pass

    def state_learning_pick_from_feeder(self, event):
        rospy.loginfo('state_learning_pick_from_feeder')
        gripper = event.kwargs.get('gripper', None)
        if gripper is None:
            self.error(severity="ERROR",
                       error=ArtBrainErrors.ERROR_LEARNING_GRIPPER_NOT_DEFINED)
            return
        result = gripper.move_to_user_client.call()

        if not result.success:
            rospy.logwarn("Can't move gripper to the user: " +
                          str(result.message))
            # TODO: inform user
        result = gripper.interaction_on_client.call()
        if not result:
            rospy.logwarn(
                "Can't change gripper interaction state: " + str(result.message))
            # TODO: check arm state, inform user

    def state_learning_pick_from_feeder_run(self, event):
        rospy.loginfo('state_learning_pick_from_feeder_run')
        rospy.sleep(2)
        self.fsm.done()

    def state_learning_pick_from_feeder_exit(self, event):
        rospy.loginfo("state_learning_pick_from_feeder_exit")
        self.left_gripper.interaction_off_client.call()
        self.left_gripper.get_ready_client.call()

    def state_learning_pick_object_id(self, event):
        rospy.loginfo('state_learning_pick_object_id')
        pass

    def state_learning_place_to_pose(self, event):
        rospy.loginfo('state_learning_place_to_pose')
        pass

    def state_learning_pick_object_id_run(self, event):
        rospy.loginfo('state_learning_pick_object_id_run')
        rospy.sleep(2)
        self.fsm.done()

    def state_learning_place_to_pose_run(self, event):
        rospy.loginfo('state_learning_place_to_pose_run')
        instruction = self.state_manager.state.program_current_item  # type: ProgramItem
        self.place_object_to_pose(instruction, update_state_manager=False, get_ready_after_place=True)

    def state_learning_wait(self, event):
        rospy.loginfo('state_learning_wait')
        pass

    def state_learning_step_error(self, event):
        rospy.loginfo('state_learning_step_error')
        severity = event.kwargs.get('severity', InterfaceState.SEVERE)
        error = event.kwargs.get('error', None)
        rospy.loginfo(severity)
        rospy.loginfo(error)
        self.state_manager.set_error(InterfaceState.INFO, error)
        self.state_manager.set_error(0, 0)
        if error is None:
            pass
            # TODO: kill brain
        if severity == InterfaceState.SEVERE:
            pass
            # TODO: kill brain
        elif severity == InterfaceState.ERROR:
            pass
        elif severity == InterfaceState.WARNING:
            if error == ArtBrainErrors.ERROR_OBJECT_MISSING or \
               error == ArtBrainErrors.ERROR_OBJECT_MISSING_IN_POLYGON:
                rospy.logwarn("Object is missing")
            elif error == ArtBrainErrors.ERROR_PICK_FAILED:
                rospy.logwarn("Pick failed")
                self.left_gripper.get_ready()
                self.left_gripper.re_init()
                self.right_gripper.get_ready()
                self.right_gripper.re_init()

        elif severity == InterfaceState.INFO:

            pass
        pass
        self.fsm.error_handled()

    def state_learning_step_done(self, event):
        rospy.loginfo('state_learning_step_done')
        self.fsm.done()
        pass

    def state_learning_done(self, event):
        rospy.loginfo('state_learning_done')
        self.fsm.done()
        pass

    # ***************************************************************************************
    #                                     MANIPULATION
    # ***************************************************************************************

    def pick_object_from_polygon(self, instruction, update_state_manager=True):
        obj = ArtBrainUtils.get_pick_obj_from_polygon(
            instruction, self.objects)
        if obj is None or obj.object_id is None:
            self.fsm.error(severity=InterfaceState.WARNING,
                           error=ArtBrainErrors.ERROR_OBJECT_MISSING_IN_POLYGON)
            if update_state_manager:
                self.state_manager.update_program_item(
                    self.ph.get_program_id(), self.block_id, instruction)
            return
        if update_state_manager:
            self.state_manager.update_program_item(
                self.ph.get_program_id(), self.block_id, instruction, {
                    "SELECTED_OBJECT_ID": obj.object_id})
        gripper = self.get_gripper(obj=obj)
        if not self.check_gripper_for_pick(gripper):
            return

        if self.pick_object_by_id(obj, gripper):
            gripper.holding_object = obj
            gripper.last_pick_instruction_id = instruction.id
            self.fsm.done(success=True)

        else:
            self.fsm.error(severity=InterfaceState.WARNING,
                           error=ArtBrainErrors.ERROR_PICK_FAILED)

    def pick_object_by_id(self, obj, gripper):

        goal = PickPlaceGoal()
        goal.object = obj.object_id
        goal.operation = goal.PICK_OBJECT_ID
        goal.keep_orientation = False
        rospy.loginfo("Picking object with ID: " + str(obj.object_id))
        gripper.pp_client.send_goal(goal)
        gripper.pp_client.wait_for_result()
        # TODO: make some error msg etc
        rospy.loginfo('got result')
        print(gripper.pp_client.get_result())
        print("status: " + gripper.pp_client.get_goal_status_text())
        print("state: " + str(gripper.pp_client.get_state()))

        if gripper.pp_client.get_result().result == 0:
            return True
        else:
            return False

    def pick_object_from_feeder(self, obj_type, gripper, pick_pose):

        goal = PickPlaceGoal()
        goal.object = obj_type
        goal.operation = goal.PICK_FROM_FEEDER
        goal.pose = pick_pose
        goal.keep_orientation = False
        rospy.loginfo("Picking object from feeder")
        gripper.pp_client.send_goal(goal)
        gripper.pp_client.wait_for_result()
        # TODO: make some error msg etc
        rospy.loginfo('got result')
        print(gripper.pp_client.get_result())
        print("status: " + gripper.pp_client.get_goal_status_text())
        print("state: " + str(gripper.pp_client.get_state()))

        if gripper.pp_client.get_result().result == 0:
            return True
        else:
            return False

    def place_object_to_pose(self, instruction, update_state_manager=True, get_ready_after_place=False):
        pose = ArtBrainUtils.get_place_pose(instruction)

        # TODO place pose

        if pose is None or len(pose) < 1:
            self.fsm.error(severity=InterfaceState.ERROR,
                           error=ArtBrainErrors.ERROR_PLACE_POSE_NOT_DEFINED)
            if update_state_manager:
                self.state_manager.update_program_item(
                    self.ph.get_program_id(), self.block_id, instruction)
            return
        else:
            if len(instruction.ref_id) < 1:
                self.fsm.error(
                    severity=InterfaceState.ERROR,
                    error=ArtBrainErrors.ERROR_NO_PICK_INSTRUCTION_ID_FOR_PLACE)
                if update_state_manager:
                    self.state_manager.update_program_item(
                        self.ph.get_program_id(), self.block_id, instruction)
                return
            rospy.logdebug(self.instruction)
            gripper = self.get_gripper_by_pick_instruction_id(
                instruction.ref_id)
            rospy.loginfo("before check")
            if not self.check_gripper_for_place(gripper):
                return
            rospy.loginfo("after check")
            if gripper.holding_object is None:
                rospy.logerr("Robot is not holding selected object")
                self.fsm.error(
                    severity=InterfaceState.WARNING,
                    error=ArtBrainErrors.ERROR_GRIPPER_NOT_HOLDING_SELECTED_OBJECT)
                if update_state_manager:
                    self.state_manager.update_program_item(
                        self.ph.get_program_id(), self.block_id, instruction)
                return
            if update_state_manager:
                self.state_manager.update_program_item(
                    self.ph.get_program_id(), self.block_id, instruction,
                    {"SELECTED_OBJECT_ID": gripper.holding_object.object_id})
            if self.place_object(gripper.holding_object, pose[0], gripper):
                gripper.holding_object = None
                # gripper.last_pick_instruction_id = self.instruction.id
                if get_ready_after_place:
                    gripper.get_ready()
                self.fsm.done(success=True)
                return
            else:
                gripper.get_ready()
                self.fsm.error(severity=InterfaceState.WARNING,
                               error=ArtBrainErrors.ERROR_PLACE_FAILED)
                return

    def place_object(self, obj, place, gripper):
        rospy.logdebug(obj)
        goal = PickPlaceGoal()
        goal.operation = goal.PLACE_TO_POSE
        goal.object = obj.object_id
        if not self.check_place_pose(place, obj):
            return False
        # TODO how to decide between 180 and 90 deg?
        # allow object to be rotated by 90 deg around z axis
        goal.z_axis_angle_increment = 3.14 / 2

        goal.pose = place
        goal.pose.header.stamp = rospy.Time.now()
        goal.pose.header.frame_id = self.objects.header.frame_id
        # TODO: how to deal with this?
        goal.pose.pose.position.z = obj.bbox.dimensions[2]/2
        rospy.loginfo("Place pose: " + str(goal.pose))
        gripper.pp_client.send_goal(goal)
        gripper.pp_client.wait_for_result()
        rospy.loginfo("Placing object with ID: " + str(obj.object_id))
        if gripper.pp_client.get_result().result == 0:
            return True
        else:
            return False

    # ***************************************************************************************
    #                                        OTHERS
    # ***************************************************************************************

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
        if self.motors_halted:
            self.fsm.error(severity=InterfaceState.WARNING,
                           error=ArtBrainErrors.ERROR_ROBOT_HALTED)
            return False
        else:
            return True

    def get_gripper(self, obj=None, pick_pose=None):

        if not self.gripper_usage == ArtGripper.GRIPPER_BOTH:
            if self.gripper_usage == ArtGripper.GRIPPER_LEFT:
                return self.left_gripper
            elif self.gripper_usage == ArtGripper.GRIPPER_RIGHT:
                return self.right_gripper

        if self.tf_listener.frameExists(
                "/base_link") and self.tf_listener.frameExists(self.objects.header.frame_id):
            if pick_pose is not None:
                transformed_pose = self.tf_listener.transformPose(
                    '/base_link', pick_pose)
                if transformed_pose.pose.position.y < 0:
                    return self.right_gripper
                else:
                    return self.left_gripper
            elif obj is not None:
                for o in self.objects.instances:
                    if o.object_id == obj.object_id:
                        obj_pose = PoseStamped()
                        obj_pose.pose = o.pose
                        obj_pose.header = self.objects.header
                        # exact time does not matter in this case
                        obj_pose.header.stamp = rospy.Time(0)
                        self.tf_listener.waitForTransform(
                            '/base_link',
                            obj_pose.header.frame_id,
                            obj_pose.header.stamp,
                            rospy.Duration(1))
                        obj_pose = self.tf_listener.transformPose(
                            '/base_link', obj_pose)
                        if obj_pose.pose.position.y < 0:
                            return self.right_gripper
                        else:
                            return self.left_gripper
        return self.left_gripper

    def get_gripper_holding_object(self, obj):
        if self.left_gripper is not None and self.left_gripper.holding_object is obj:
            return self.left_gripper
        elif self.right_gripper is not None and self.right_gripper.holding_object is obj:
            return self.right_gripper
        else:
            return None

    def get_gripper_by_pick_instruction_id(self, pick_instruction_ids):

        if self.left_gripper is not None and self.left_gripper.last_pick_instruction_id in pick_instruction_ids:
            return self.left_gripper
        elif self.right_gripper is not None and self.right_gripper.last_pick_instruction_id in pick_instruction_ids:
            return self.right_gripper
        else:
            return None

    def get_gripper_path_following(self):
        if not self.gripper_usage == ArtGripper.GRIPPER_BOTH:
            if self.gripper_usage == ArtGripper.GRIPPER_LEFT:
                return self.left_gripper
            elif self.gripper_usage == ArtGripper.GRIPPER_RIGHT:
                return self.right_gripper
        else:
            return self.right_gripper

    def get_gripper_welding_points(self):
        if not self.gripper_usage == ArtGripper.GRIPPER_BOTH:
            if self.gripper_usage == ArtGripper.GRIPPER_LEFT:
                return self.left_gripper
            elif self.gripper_usage == ArtGripper.GRIPPER_RIGHT:
                return self.right_gripper
        else:
            return self.right_gripper

    def get_gripper_welding_seam(self):
        if not self.gripper_usage == ArtGripper.GRIPPER_BOTH:
            if self.gripper_usage == ArtGripper.GRIPPER_LEFT:
                return self.left_gripper
            elif self.gripper_usage == ArtGripper.GRIPPER_RIGHT:
                return self.right_gripper
        else:
            return self.right_gripper

    def get_gripper_drill_points(self):
        if not self.gripper_usage == ArtGripper.GRIPPER_BOTH:
            if self.gripper_usage == ArtGripper.GRIPPER_LEFT:
                return self.left_gripper
            elif self.gripper_usage == ArtGripper.GRIPPER_RIGHT:
                return self.right_gripper
        else:
            return self.right_gripper

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
            self.fsm.error(severity=InterfaceState.SEVERE,
                           error=ArtBrainErrors.ERROR_NO_GRIPPER_AVAILABLE)
            return False

        if gripper.pp_client is None:
            rospy.logerr("No pick place client!")
            self.fsm.error(severity=InterfaceState.SEVERE,
                           error=ArtBrainErrors.ERROR_GRIPPER_PP_CLIENT_MISSING)
            return False

        if not gripper.pp_client.wait_for_server(rospy.Duration(2)):
            rospy.logerr("Pick place server is not running!")
            self.fsm.error(severity=InterfaceState.WARNING,
                           error=ArtBrainErrors.ERROR_PICK_PLACE_SERVER_NOT_READY)
            return False

        return True

    def check_gripper_for_place(self, gripper):
        if not self.check_gripper(gripper):
            return False

        if gripper.holding_object is None:
            rospy.logwarn("Place: gripper " + gripper.name +
                          " is not holding object")
            self.fsm.error(severity=InterfaceState.WARNING,
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
            self.fsm.error(severity=InterfaceState.WARNING,
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

        resp = startProgramResponse()

        if not self.is_everything_calibrated():
            resp.success = False
            resp.error = 'Something is not calibrated'
            rospy.loginfo('Something is not calibrated')
            return resp

        if self.executing_program:

            resp.success = False
            resp.error = 'Program already running'
            return resp

        if not self.fsm.is_waiting_for_action():
            resp.success = False
            resp.error = 'Not ready for program start!'
            return resp

        rospy.loginfo('Loading program ' + str(req.program_id) + ' from db...')

        program = self.art.load_program(req.program_id)

        if not self.ph.load(program):
            resp.success = False
            resp.error = 'Cannot get program'
            return resp

        rospy.loginfo('Starting program')

        rospy.Timer(rospy.Duration(
            1), self.program_start_timer_cb, oneshot=True)
        rospy.loginfo("program started")
        resp.success = True
        return resp

    def program_stop_cb(self, req):
        resp = TriggerResponse()
        resp.success = True
        if self.executing_program:
            rospy.loginfo('Stopping program')
            self.executing_program = False
        else:
            resp.success = False
            resp.message = "Program is not running"

        return resp

    def program_error_response_cb(self, req):
        resp = ProgramErrorResolveResponse()
        if not self.fsm.is_program_error():
            resp.success = False
            return resp
        rospy.loginfo('We\'ve got response to our program error')
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
        resp = TriggerResponse()
        if not self.is_everything_calibrated():
            resp.success = False
            resp.message = 'Something is not calibrated'
            rospy.loginfo('Something is not calibrated')
            return resp

        if not self.fsm.is_waiting_for_action():
            resp.success = False
            resp.message = 'Not ready for learning start!'
            rospy.loginfo('Not ready for learning start!')
            return resp

        rospy.loginfo('Starting learning')
        resp.success = True
        self.fsm.learning_start()
        return resp

    def learning_stop_cb(self, req):
        resp = TriggerResponse()
        if not self.fsm.is_learning_run:
            resp.success = False
        rospy.loginfo('Stopping learning')
        self.learning = False

        resp.success = True
        self.fsm.learning_done()
        return resp

    def interface_state_manager_cb(self,
                                   state,  # type: InterfaceState
                                   msg,  # type: InterfaceState
                                   flags):
        '''
        if state.system_state == InterfaceState.STATE_LEARNING:
            if "learning" not in self.state:
                if self.fsm.is_waiting_for_action():
                    pass
                    # TODO: handle
            if self.fsm.is_learning_run():
                if msg.program_current_item.type == ProgramItem.MANIP_PICK:
                    self.pick(from_feeder=False)
                    # TODO: check flags for from_feeder
                elif msg.program_current_item.type == ProgramItem.MANIP_PICK_PLACE:
                    self.pick_place(from_feeder=False)
                elif msg.program_current_item.type == ProgramItem.MANIP_PLACE:
                    self.place()
                elif msg.program_current_item.type == ProgramItem.WAIT:
                    self.wait()
            if (self.fsm.is_learning_pick() and msg.program_current_item.type != ProgramItem.MANIP_PICK) or \
                    (self.fsm.is_learning_place() and msg.program_current_item.type != ProgramItem.MANIP_PICK_PLACE) or \
                    (self.fsm.is_learning_pick_place() and msg.program_current_item.type != ProgramItem.MANIP_PICK_PLACE):
                self.done()
                return

            # if self.is_learning_pick_from_feeder():
        '''
        pass

    def motors_halted_cb(self, req):
        if not self.initialized:
            return
        if self.motors_halted and not req.data:
            if self.gripper_usage == ArtGripper.GRIPPER_LEFT:
                self.left_gripper.get_ready()
            elif self.gripper_usage == ArtGripper.GRIPPER_RIGHT:
                self.right_gripper.get_ready()
            elif self.gripper_usage == ArtGripper.GRIPPER_BOTH:
                self.left_gripper.get_ready()
                self.right_gripper.get_ready()
        self.motors_halted = req.data

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

    def learning_request_cb(self,
                            goal):  # type: LearningRequestGoal
        result = LearningRequestResult()
        if not self.fsm.is_learning_run:
            result.success = False
            result.message = "Not in learning mode!"
        rospy.loginfo("Learning_request goal: " + str(goal.request))

        instruction = self.state_manager.state.program_current_item  # type: ProgramItem

        if goal.request == LearningRequestGoal.GET_READY:
            if self.fsm.is_learning_run:
                if instruction.type == instruction.PICK_OBJECT_ID:
                    self.fsm.pick_object_id()
                    pass
                elif instruction.type == instruction.PICK_FROM_FEEDER:
                    self.fsm.pick_from_feeder(gripper=self.left_gripper)
                    # TODO: choose which gripper use
                    # TODO: check if it worked
                elif instruction.type == instruction.PICK_FROM_POLYGON:
                    self.fsm.pick_from_polygon()
                    pass
                elif instruction.type == instruction.PLACE_TO_POSE:
                    self.fsm.place_to_pose()
            else:
                result.success = False
                result.message = "Not in learning state!"
                self.as_learning_request.set_aborted(result)
                return
                # TODO: handle error
        elif goal.request == LearningRequestGoal.EXECUTE_ITEM:
            # self.fsm.error(severity=InterfaceState.INFO,
            #                error=InterfaceState.ERROR_LEARNING_NOT_IMPLEMENTED)
            if self.fsm.is_learning_run:
                if instruction.type == instruction.PICK_OBJECT_ID:
                    self.fsm.pick_object_id_run()
                    pass
                elif instruction.type == instruction.PICK_FROM_FEEDER:
                    self.fsm.pick_from_feeder_run(gripper=self.left_gripper)
                    # TODO: choose which gripper use
                    # TODO: check if it worked
                elif instruction.type == instruction.PICK_FROM_POLYGON:
                    self.fsm.pick_from_polygon_run()
                    pass
                elif instruction.type == instruction.PLACE_TO_POSE:
                    self.fsm.place_to_pose_run()
            else:
                result.success = False
                result.message = "Not in learning state!"
                self.as_learning_request.set_aborted(result)
                return
            return
            pass
        elif goal.request == LearningRequestGoal.DONE:
            # Great!
            self.fsm.done()
            pass

        result.success = True
        self.as_learning_request.set_succeeded(result)


if __name__ == '__main__':
    rospy.init_node('new_art_brain', log_level=rospy.DEBUG)

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
