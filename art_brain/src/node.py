#!/usr/bin/env python
# coding=utf-8

import rospy
import time
import copy

import actionlib
from art_msgs.msg import LocalizeAgainstUMFAction, LocalizeAgainstUMFGoal, LocalizeAgainstUMFResult
from std_srvs.srv import Empty, EmptyRequest, EmptyResponse
from art_msgs.msg import UserStatus,  UserActivity, InterfaceState
from art_msgs.srv import startProgram,  startProgramResponse,  getProgram
from geometry_msgs.msg import PoseStamped, Pose
from std_msgs.msg import String, Bool
from art_msgs.msg import pickplaceAction, pickplaceGoal, SystemState, ObjInstance, InstancesArray, ProgramItem, \
    ObjectType
from shape_msgs.msg import SolidPrimitive
from art_msgs.srv import getObjectType
import matplotlib.path as mplPath
import numpy as np
import random
from art_utils import InterfaceStateManager,  ArtApiHelper,  ProgramHelper

from tf import TransformerROS, TransformListener

from transitions import Machine
# from transitions.extensions import GraphMachine as Machine
from transitions import State

import logging
from transitions import logger

from art_brain.brain_utils import ArtBrainUtils, ArtGripper


# TODO:
# pause/resume programu
# manequin mod pro gravitation feeder
# při pick object zkontrolovat pick_pose a pripadně použít
# používat obě ramena robota -> done, otestovat na stole
# při place zkontrolovat place pose (jestli tam není jiný objekt) -> done, otestovat na stole
# při nepovedené operaci se zeptat usera jestli má zopakovat instrukci, pokračovat nebo skončit

class ArtBrainMachine(object):

    states = [State(name='pre_init', on_enter=[],  on_exit=[]),
              State(name='init', on_enter=['state_init_ros'],  on_exit=[]),
              State(name='waiting_for_action', on_enter=[
                    'state_waiting_for_action'], on_exit=[]),
              State(name='program_init', on_enter=[
                    'state_program_init'],  on_exit=[]),
              State(name='program_run', on_enter=[
                    'state_program_run'],  on_exit=[]),
              State(name='get_ready', on_enter=[
                    'state_get_ready'],  on_exit=[]),
              State(name='pick', on_enter=['state_pick'],  on_exit=[]),
              State(name='place', on_enter=['state_place'],  on_exit=[]),
              State(name='pick_place',   on_enter=[
                    'state_pick_place'],  on_exit=[]),
              State(name='wait', on_enter=['state_wait'],  on_exit=[]),
              State(name='program_error', on_enter=[
                    'state_program_error'],  on_exit=[]),
              State(name='program_finished', on_enter=[
                    'state_program_finished'],  on_exit=[]),
              State(name='program_load_instruction', on_enter=[
                    'state_program_load_instruction'],  on_exit=[]),
              State(name='teaching_init', on_enter=['state_teaching_init'],  on_exit=[])]

    # ROS Connections
    show_marker_service = None
    hide_marker_service = None
    table_localize_action = None
    pr2_localize_action = None
    calibrate_pr2 = None
    calibrate_table = None
    user_status_sub = None
    user_activity_sub = None
    table_calibrated_sub = None
    table_calibrating_sub = None
    system_calibrated_sub = None
    srv_program_start = None
    srv_program_stop = None
    srv_program_pause = None
    srv_program_resume = None
    calibrate_table_srv_client = None
    get_obj_type_srv_client = None
    state_manager = None
    art = None
    ph = None
    user_activity = None
    objects_sub = None
    state_publisher = None
    pp_client = None

    left_gripper = None
    right_gripper = None

    gripper_usage = ArtGripper.GRIPPER_BOTH

    transformer = TransformerROS()
    tf_listener = TransformListener()

    block_id = None
    user_id = 0
    objects = InstancesArray()
    executing_program = False
    instruction = None
    holding_object_left = None
    holding_object_right = None
    stop_server = False
    recalibrate = False
    table_calibrated = False
    table_calibrating = False
    cells_calibrated = False
    system_calibrated = False

    quit = False

    def __init__(self):
        self.name = 'brain'

        self.machine = Machine(model=self,  states=ArtBrainMachine.states, initial='pre_init',
                               auto_transitions=False, send_event=True, queued=True)

        # *** transitions ***

        self.machine.add_transition('init',  'pre_init',  'init')
        self.machine.add_transition(
            'init_done',  'init',  'waiting_for_action', conditions='is_everything_calibrated')

        # program
        self.machine.add_transition(
            'program_start',  'waiting_for_action',  'program_init')
        self.machine.add_transition(
            'program_init_done',  'program_init',  'program_run')
        self.machine.add_transition('error',  'program_init',  'program_error')
        self.machine.add_transition('error',  'program_run',  'program_error')
        self.machine.add_transition(
            'program_error_handled', 'program_error',  'waiting_for_action')
        self.machine.add_transition(
            'done', 'program_load_instruction',  'program_run')
        self.machine.add_transition(
            'error', 'program_load_instruction',  'program_error')
        self.machine.add_transition(
            'finished', 'program_load_instruction',  'program_finished')
        self.machine.add_transition(
            'done', 'program_finished',  'waiting_for_action')

        # get ready instruction
        self.machine.add_transition('get_ready', 'program_run', 'get_ready')
        self.machine.add_transition(
            'done', 'get_ready', 'program_load_instruction')
        self.machine.add_transition('error', 'get_ready', 'program_error')

        # pick instruction
        self.machine.add_transition('pick',  'program_run',  'pick')
        self.machine.add_transition(
            'done',  'pick',  'program_load_instruction')
        self.machine.add_transition('error',  'pick',  'program_error')

        # place instruction
        self.machine.add_transition('place',  'program_run',  'place')
        self.machine.add_transition(
            'done',  'place',  'program_load_instruction')
        self.machine.add_transition('error',  'place',  'program_error')

        # pick_place instruction
        self.machine.add_transition(
            'pick_place',  'program_run',  'pick_place')
        self.machine.add_transition(
            'done',  'pick_place',  'program_load_instruction')
        self.machine.add_transition('error',  'pick_place',  'program_error')

        # wait instruction
        self.machine.add_transition('wait',  'program_run',  'wait')
        self.machine.add_transition(
            'done',  'wait',  'program_load_instruction')
        self.machine.add_transition('error',  'wait',  'program_error')

        # self.machine.graph.draw('my_state_diagram.png', prog='dot')

        logger.setLevel(logging.DEBUG)

        self.init()

    # ***************************************************************************************
    #                                       STATES
    # ***************************************************************************************

    def state_init_ros(self,  event):
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
        self.table_calibrated_sub = rospy.Subscriber("/art/interface/touchtable/calibrated", Bool,
                                                     self.table_calibrated_cb)
        self.table_calibrating_sub = rospy.Subscriber("/art/interface/touchtable/calibrating", Bool,
                                                      self.table_calibrating_cb)
        self.system_calibrated_sub = rospy.Subscriber("/system_calibrated", Bool,
                                                      self.system_calibrated_cb)

        self.srv_program_start = rospy.Service(
            '/art/brain/program/start', startProgram, self.program_start_cb)
        self.srv_program_stop = rospy.Service(
            '/art/brain/program/stop', Empty, self.program_stop_cb)

        # self.srv_program_pause = rospy.Service(/art/brain/program/pause', Empty, self.program_pause_cb)
        # self.srv_program_resume = rospy.Service(/art/brain/program/resume',
        # Empty, self.program_resume_cb)

        self.state_manager = InterfaceStateManager(
            InterfaceState.BRAIN_ID)  # TODO callback?
        self.state_manager.set_system_state(
            InterfaceState.STATE_INITIALIZING)

        self.art = ArtApiHelper(brain=True)
        self.ph = ProgramHelper()

        self.objects_sub = rospy.Subscriber(
            "/art/object_detector/object_filtered", InstancesArray, self.objects_cb)

        # TODO use this topic instead of system_state in InterfaceState (duplication) ??
        # TODO move (pub/sub) to InterfaceStateManager?
        self.state_publisher = rospy.Publisher(
            "/art/brain/system_state", SystemState, queue_size=1)

        gripper = rospy.get_param('gripper_usage', 'both')
        if gripper == 'left':
            self.gripper_usage = ArtGripper.GRIPPER_LEFT
        elif gripper == 'right':
            self.gripper_usage = ArtGripper.GRIPPER_RIGHT
        elif gripper == 'both':
            self.gripper_usage = ArtGripper.GRIPPER_BOTH

        if self.gripper_usage == ArtGripper.GRIPPER_BOTH or ArtGripper.GRIPPER_LEFT:
            self.left_gripper = ArtGripper('left_arm', '/art/pr2/left_arm/pp')
        else:
            self.left_gripper = None
        if self.gripper_usage == ArtGripper.GRIPPER_BOTH or ArtGripper.GRIPPER_RIGHT:
            self.right_gripper = ArtGripper('right_arm', '/art/pr2/right_arm/pp')
        else:
            self.right_gripper = None

        self.art.wait_for_api()

        self.get_obj_type_srv_client = rospy.ServiceProxy(
            '/art/db/object_type/get', getObjectType)

        if not self.table_calibrated:
            rospy.loginfo(
                'Waiting for /art/interface/touchtable/calibrate service')
            rospy.wait_for_service('/art/interface/touchtable/calibrate')
            rospy.loginfo(
                'Get /art/interface/touchtable/calibrate service')
            self.calibrate_table_srv_client = rospy.ServiceProxy(
                '/art/interface/touchtable/calibrate', Empty)
            attempt = 1
            rospy.sleep(1)
            while not self.table_calibrated:
                rospy.loginfo("Trying to calibrate table, attempt " + str(attempt))
                self.calibrate_table_srv_client.call()
                rospy.sleep(1)
                attempt += 1
                while not self.table_calibrated and self.table_calibrating:
                    rospy.sleep(1)

        r = rospy.Rate(1)
        while not self.is_everything_calibrated():
            if rospy.is_shutdown():
                return
            rospy.loginfo("Waiting for calibration")
            r.sleep()
        self.init_done()

    def state_program_init(self,  event):
        rospy.loginfo('state_program_init')
        rospy.loginfo('New program ready!')

        if self.ph is None:
            self.failure()
            return

        (self.block_id, item_id) = self.ph.get_first_item_id()
        self.instruction = self.ph.get_item_msg(self.block_id, item_id)

        self.executing_program = True

        self.state_manager.set_system_state(
            InterfaceState.STATE_PROGRAM_RUNNING)
        self.program_init_done()

    def state_program_load_instruction(self, event):
        rospy.loginfo('state_program_load_instruction')
        pass

    def state_program_run(self,  event):
        rospy.loginfo('state_program_run')
        if self.instruction is None:
            self.failure()
            return

        rospy.logdebug(
            'Program id: ' + str(self.ph.get_program_id()) + ', item id: ' + str(self.instruction.id) +
            ', item type: ' + str(self.instruction.type))

        instructions = {
            ProgramItem.GET_READY: self.get_ready,
            ProgramItem.MANIP_PICK: self.pick,
            ProgramItem.MANIP_PLACE: self.place,
            ProgramItem.MANIP_PICK_PLACE: self.pick_place,
            ProgramItem.WAIT: self.wait,
        }

        instruction_transition = instructions.get(self.instruction.type, None)
        print (instruction_transition)
        if instruction_transition is None:
            self.error()
            return
        instruction_transition()

        if not self.executing_program:
            self.failure()
            return

    def state_pick(self, event):
        """

        :type instruction: ProgramItem
        :return:
        """
        rospy.loginfo('state_pick')

        obj = ArtBrainUtils.get_pick_obj(self.instruction, self.objects)
        self.state_manager.update_program_item(self.ph.get_program_id(), self.block_id, self.instruction,
                                               {"SELECTED_OBJECT_ID": obj.object_id})

        picking_from_feeder = False
        if self.ph.is_pick_pose_set(self.block_id, self.instruction.id):
            picking_from_feeder = True

        if picking_from_feeder is False and obj is None or obj.object_id is None:
            rospy.logerr("Object not specified")
            self.done(success=False)

            return
        if picking_from_feeder:
            gripper = self.get_gripper(pick_pose=self.instruction.pick_pose)
        else:
            gripper = self.get_gripper(obj=obj)
        if gripper.pp_client is None:
            rospy.logerr("No pick place client!")
            self.done(success=False)
            return

        if gripper.holding_object is not None:
            rospy.logwarn("Robot is holding object in selected gripper, let's try another gripper")
            if gripper is self.left_gripper:
                gripper = self.right_gripper
            else:
                gripper = self.left_gripper
            if gripper.holding_object:
                rospy.logerr("Robot is holding objects in both grippers!")
                self.done(success=False)
                return
        if self.pick_object(obj, gripper.pp_client, self.instruction.pick_pose):
            gripper.holding_object = obj
            self.done(success=True)

        else:
            self.done(success=False)

    def state_place(self, event):
        """

        :type instruction: ProgramItem
        :return:
        """
        rospy.loginfo('state_place')
        pose = ArtBrainUtils.get_place_pose(self.instruction)
        self.state_manager.update_program_item(
            self.ph.get_program_id(), self.block_id, self.instruction)
        # TODO place pose
        obj = copy.deepcopy(self.instruction.object)

        if pose is None:
            self.done(success=False)
            return
        else:
            gripper = self.get_gripper_holding_object(obj)
            if gripper.holding_object is None:
                rospy.logerr("Robot is not holding selected object")

            if gripper.pp_client is None:
                rospy.logerr("No pick place client!")
                self.done(success=False)
                return

            if self.place_object(gripper.holding_object, pose, gripper.pp_client):
                gripper.holding_object = None
                self.done(success=True)
                return
            else:
                self.done(success=False)
                return

    def state_pick_place(self, event):
        rospy.loginfo('state_pickplace')
        obj = ArtBrainUtils.get_pick_obj(self.instruction, self.objects)
        pose = ArtBrainUtils.get_place_pose(self.instruction)

        self.state_manager.update_program_item(self.ph.get_program_id(), self.block_id, self.instruction,
                                               {"SELECTED_OBJECT_ID": obj.object_id})
        # TODO also update p.i. with selected place pose when not given (place
        # polygon)
        picking_from_feeder = False
        if self.ph.is_pick_pose_set(self.block_id, self.instruction.id):
            picking_from_feeder = True

        if picking_from_feeder is False and obj is None or obj.object_id is None or pose is None:
            rospy.logerr('could not get obj_id or pose')

            self.done(success=False)
            return
        if picking_from_feeder:
            gripper = self.get_gripper(pick_pose=self.instruction.pick_pose)
        else:
            gripper = self.get_gripper(obj=obj)
        if gripper.pp_client is None:
            rospy.logerr("No pick place client!")
            self.done(success=False)
            return
        if gripper.holding_object is not None:
            rospy.logwarn("Robot is holding object in selected gripper, let's try another gripper")
            if gripper is self.left_gripper:
                gripper = self.right_gripper
            else:
                gripper = self.left_gripper
            if gripper.holding_object:
                rospy.logerr("Robot is holding objects in both grippers!")
                self.done(success=False)
                return
        if self.pick_place_object(obj, pose, gripper.pp_client):
            self.done(success=True)
            return

        self.done(success=False)

    def state_wait(self, event):
        """

                :type instruction: ProgramItem
                :return:
                """

        rospy.loginfo('state_wait')

        self.state_manager.update_program_item(
            self.ph.get_program_id(), self.block_id, self.instruction)

        rate = rospy.Rate(10)

        if self.instruction.spec == self.instruction.WAIT_FOR_USER:
            while self.user_activity != UserActivity.READY:
                rate.sleep()
        elif self.instruction.spec == self.instruction.WAIT_UNTIL_USER_FINISHES:
            while self.user_activity != UserActivity.WORKING:
                rate.sleep()
        else:
            self.done(success=False)
            return

        self.done(success=True)

    def state_get_ready(self, event):
        rospy.loginfo('state_get_ready')
        self.state_manager.update_program_item(
            self.ph.get_program_id(), self.block_id, self.instruction)
        # TODO: call some service to set PR2 to ready position
        self.done(success=True)

    def state_program_load_instruction(self, event):
        rospy.loginfo('state_program_load_instruction')
        success = event.kwargs.get('success', False)
        if success:
            (self.block_id, item_id) = self.ph.get_id_on_success(
                self.block_id, self.instruction.id)
        else:
            (self.block_id, item_id) = self.ph.get_id_on_failure(
                self.block_id, self.instruction.id)

        if self.block_id == 0:
            self.finished()
            return

        self.instruction = self.ph.get_item_msg(self.block_id, item_id)

        if self.instruction is None:
            self.executing_program = False
            self.error()
            return
        self.done()

    def state_program_finished(self, event):
        rospy.loginfo('state_program_finished')
        self.state_manager.set_system_state(
            InterfaceState.STATE_PROGRAM_FINISHED)
        self.state_manager.send()
        self.executing_program = False
        self.done()

    def state_program_error(self, event):
        rospy.loginfo('state_program_error')
        self.executing_program = False
        self.program_error_handled()

    def state_waiting_for_action(self, event):
        rospy.loginfo('state_waiting_for_action')
        self.state_manager.set_system_state(InterfaceState.STATE_IDLE)

    def teaching_init(self, event):
        pass

    # ***************************************************************************************
    #                                     MANIPULATION
    # ***************************************************************************************

    def pick_object(self, obj, pp_client, pick_pose=None):

        goal = pickplaceGoal()
        goal.id = obj.object_id
        goal.operation = goal.PICK
        goal.keep_orientation = False
        rospy.loginfo("Picking object with ID: " + str(obj.object_id))
        pp_client.send_goal(goal)
        pp_client.wait_for_result()
        # TODO: make some error msg etc
        rospy.loginfo('got result')
        print(pp_client.get_result())
        print("status: " + pp_client.get_goal_status_text())
        print("state: " + str(pp_client.get_state()))

        if pp_client.get_result().result == 0:
            return True
        else:
            return False

    def place_object(self, obj, place, pp_client):

        goal = pickplaceGoal()
        goal.operation = goal.PLACE
        goal.id = obj.object_id
        if not self.check_place_pose(place, obj):
            return False
        # TODO how to decide between 180 and 90 deg?
        # allow object to be rotated by 90 deg around z axis
        goal.z_axis_angle_increment = 3.14 / 2

        goal.place_pose = PoseStamped()

        goal.place_pose = place
        goal.place_pose.header.stamp = rospy.Time.now()
        goal.place_pose.header.frame_id = self.objects.header.frame_id
        # TODO: how to deal with this?
        goal.place_pose.pose.position.z = 0.1  # + obj.bbox.dimensions[2]/2
        rospy.loginfo("Place pose: " + str(goal.place_pose))
        pp_client.send_goal(goal)
        pp_client.wait_for_result()
        rospy.loginfo("Placing object with ID: " + str(obj.object_id))
        if pp_client.get_result().result == 0:
            return True
        else:
            return False

    def pick_place_object(self,  obj,  place, pp_client, pick_pose=None):
        goal = pickplaceGoal()
        goal.id = obj.object_id
        if not self.check_place_pose(place, obj):
            return False
        goal.operation = goal.PICK_AND_PLACE
        goal.keep_orientation = False
        rospy.loginfo("Picking and placing object with ID: " + str(obj.object_id))

        # allow object to be rotated by 90 deg around z axis
        goal.z_axis_angle_increment = 3.14 / 2

        goal.place_pose = PoseStamped()

        goal.place_pose = place
        goal.place_pose.header.stamp = rospy.Time.now()
        # TODO: how to deal with this?
        goal.place_pose.pose.position.z = 0.1  # + obj.bbox.dimensions[2]/2
        rospy.loginfo("Place pose: " + str(goal.place_pose))
        pp_client.send_goal(goal)
        pp_client.wait_for_result()
        if pp_client.get_result().result == 0:
            return True
        else:
            return False

    # ***************************************************************************************
    #                                        OTHERS
    # ***************************************************************************************

    def is_table_calibrated(self):
        return self.table_calibrated

    def is_everything_calibrated(self, event=None):

        return self.table_calibrated and self.system_calibrated

    def get_gripper(self, obj=None, pick_pose=None):

        if not self.gripper_usage == ArtGripper.GRIPPER_BOTH:
            if self.gripper_usage == ArtGripper.GRIPPER_LEFT:
                return self.left_gripper
            elif self.gripper_usage == ArtGripper.GRIPPER_RIGHT:
                return self.right_gripper

        if self.tf_listener.frameExists("/base_link") and self.tf_listener.frameExists(self.objects.header.frame_id):
            if pick_pose is not None:
                transformed_pose = self.tf_listener.transformPose('/base_link', pick_pose)
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
                        obj_pose = self.tf_listener.transformPose('/base_link', obj_pose)
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

    # ***************************************************************************************
    #                                     ROS COMMUNICATION
    # ***************************************************************************************

    def program_start_cb(self,  req):

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

        rospy.loginfo('Loading program ' + str(req.program_id) + ' from db...')

        program = self.art.load_program(req.program_id)

        if not self.ph.load(program):
            resp.success = False
            resp.error = 'Cannot get program'
            return resp

        rospy.loginfo('Starting program')

        self.program_start()
        resp.success = True
        return resp

    def program_stop_cb(self,  req):

        rospy.loginfo('Stopping program ' + str(req.program_id) + '...')
        self.executing_program = False
        return EmptyResponse()

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

    def system_calibrated_cb(self,  req):
        self.system_calibrated = req.data

    def get_object_max_width(self, obj):
        if obj is None:
            rospy.logerr('No object is specified')
            return None
        try:
            obj_type = self.get_obj_type_srv_client.call(obj.object_type)
            if not obj_type.success:
                rospy.logerr('No object with id ' + str(obj.object_id) + ' found')
                return None
            if obj_type.object_type.bbox.type != SolidPrimitive.BOX:
                rospy.logerr('Sorry, only BOX type objects are supported at the moment')
            x = obj_type.object_type.bbox.dimensions[SolidPrimitive.BOX_X]
            y = obj_type.object_type.bbox.dimensions[SolidPrimitive.BOX_Y]
            return np.hypot(x/2, y/2)
        except rospy.ServiceException, e:
            rospy.logerr('Service call failed: ' + str(e))
            return None


if __name__ == '__main__':
    rospy.init_node('new_art_brain', log_level=rospy.INFO)

    try:
        node = ArtBrainMachine()

        rate = rospy.Rate(100)

        while not rospy.is_shutdown():
            # node.process()
            rate.sleep()
    except rospy.ROSInterruptException:
        pass
