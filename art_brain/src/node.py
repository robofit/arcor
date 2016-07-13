#!/usr/bin/env python

import rospy
import time

import actionlib
from art_msgs.msg import RobotProgramAction, RobotProgramFeedback,  RobotProgramResult, RobotProgramActionGoal
from art_msgs.msg import LocalizeAgainstUMFAction, LocalizeAgainstUMFGoal, LocalizeAgainstUMFResult
from std_srvs.srv import Empty, EmptyRequest, EmptyResponse
from art_msgs.msg import UserStatus
from geometry_msgs.msg import PoseStamped, Pose
from std_msgs.msg import String
from art_msgs.msg import pickplaceAction, pickplaceGoal, SystemState, ObjInstance, InstancesArray, ProgramItem
import matplotlib.path as mplPath
import numpy as np
import random

class ArtBrain:
    UNKNOWN = -2  # should not happen!
    NOP = -1  # no operation
    GET_READY = 0  # retract arms etc.
    MANIP_PICK = 1
    MANIP_PLACE = 2
    MANIP_PICK_PLACE = 3
    WAIT = 4

    INST_OK = 100
    INST_BAD_DATA = 101
    INST_FAILED = 102

    SYSTEM_UNKNOWN = 0
    SYSTEM_START = 1
    SYSTEM_CALIBRATING = 2
    SYSTEM_STARTING_PROGRAM_SERVER = 3
    SYSTEM_READY_FOR_PROGRAM_REQUESTS = 4
    SYSTEM_STOPPING_PROGRAM_SERVER = 5


    def __init__(self):
        self.show_marker_service = rospy.get_param('show_marker_service', '/art/projected_gui/show_marker')
        self.hide_marker_service = rospy.get_param('hide_marker_service', '/art/projected_gui/hide_marker')
        self.table_localize_action = rospy.get_param('table_localize_action', '/umf_localizer_node_table/localize')
        self.pr2_localize_action = rospy.get_param('pr2_localize_action', '/umf_localizer_node/localize')

        self.calibrate_pr2 = rospy.get_param('calibrate_pr2', False)
        self.calibrate_table = rospy.get_param('calibrate_table', False)

        self.user_status_sub = rospy.Subscriber("/art/user/status", UserStatus, self.user_status_cb)
        self.object_to_pick_sub = rospy.Subscriber("/art/projected_gui/selected_object", String, self.selected_object_cb)
        self.pose_to_place_sub = rospy.Subscriber("/art/projected_gui/selected_place", PoseStamped, self.selected_place_cb)
        self.objects_sub = rospy.Subscriber("/art/object_detector/object_filtered", InstancesArray, self.objects_cb)

        self.state_publisher = rospy.Publisher("/art/brain/system_state", SystemState, queue_size=1)

        self.pp_client = actionlib.SimpleActionClient('/pr2_pick_place_left/pp', pickplaceAction)

        self.state = self.SYSTEM_START
        self.user_id = 0
        self.selected_object = None  # type: str
        self.selected_object_last_update = None  # type: rospy.Time
        self.selected_place = None  # type: PoseStamped
        self.objects = InstancesArray()
        self.executing_program = False

        self.instruction = None
        self.holding_object = None
        self.stop_server = False
        self.recalibrate = False

        self.prog_as = None
        self.quit = False
        
        self.prog_id = None
        self.it_id = None

    def send_feedback(self,  object_id = ''):
        
        feedback = RobotProgramFeedback()
        feedback.current_program = self.prog_id
        feedback.current_item = self.it_id
        feedback.object = object_id
        self.prog_as.publish_feedback(feedback)

    def execute_cb(self, goal):
        """

        :type goal:
        :return:
        """
        if self.executing_program:
            res = RobotProgramResult()
            res.result = res.BUSY
            self.prog_as.set_aborted(res)
            return

        self.executing_program = True
        for prog in goal.program_array.programs:
            program_end = False

            # for it in prog.items:
            it = prog.items[0]
            while not program_end:
                
                self.prog_id = prog.id
                self.it_id = it.id
                
                self.send_feedback()
                
                rospy.loginfo('Program id: ' + str(prog.id) + ', item id: ' + str(it.id) + ', item type: ' + str(it.type))

                self.instruction = it.type
                instruction_function = self.instruction_switcher()
                result = instruction_function(it)
                if result == self.INST_OK:
                    it = self.get_item_by_id(prog, it.on_success)
                elif result == self.INST_BAD_DATA or result == self.INST_FAILED:
                    it = self.get_item_by_id(prog, it.on_failure)
                else:
                    it = None

                if it is None:
                    res = RobotProgramResult()
                    res.result = RobotProgramResult.FAILURE
                    self.prog_as.set_aborted(res)
                    self.executing_program = False
                    return

        res = RobotProgramResult()
        res.result = RobotProgramResult.SUCCESS
        self.prog_as.set_succeeded(res)
        self.executing_program = False

    def instruction_switcher(self):
        instructions = {
            self.NOP: self.nop,
            self.GET_READY: self.get_ready,
            self.MANIP_PICK: self.manip_pick,
            self.MANIP_PLACE: self.manip_place,
            self.MANIP_PICK_PLACE: self.manip_pick_place,
            self.WAIT: self.wait,

        }
        return instructions.get(self.instruction, self.unknown_instruction)

    @staticmethod
    def get_item_by_id(program, item_id):
        for it in program.items:
            if it.id == item_id:
                return it
        return None

    def get_ready(self, instruction):
        # TODO: call some service to set PR2 to ready position
        return self.INST_OK

    def get_pick_obj_id(self, instruction):
        if instruction.spec == instruction.MANIP_ID:
            obj_id = instruction.object
        elif instruction.spec == instruction.MANIP_TYPE:

            pick_polygon = []
            pol = None
            for point in instruction.pick_polygon: # TODO check frame_id and transform to table frame?
                pick_polygon.append([point.point.x,  point.point.y])
            if len(pick_polygon) > 0:
                pol = mplPath.Path(np.array(pick_polygon))

            # shuffle the array to not get the same object each time
            random.shuffle(self.objects.instances)

            for obj in self.objects.instances:
                
                if pol is None:
                    
                    # if no pick polygon is specified - let's take the first object of that type
                    if obj.object_type == instruction.object:
                        obj_id = obj.object_id
                        break
                        
                else:
                    
                    # test if some object is in polygon and take the first one
                    if pol.contains_point([obj.pose.position.x,  obj.pose.position.y]):
                        obj_id = obj.object_id
                        rospy.loginfo('Selected object: ' + obj_id)
                        break
                    
            else:
                if pol is not None:
                    rospy.loginfo('No object in the specified polygon')
                    print pol
                return self.INST_BAD_DATA
        else:
            return self.INST_BAD_DATA
        return obj_id

    def get_place_pose(self, instruction):
        if self.holding_object is None:
            return None
        if instruction.spec == instruction.MANIP_ID:
            pose = instruction.place_pose
        elif instruction.spec == instruction.MANIP_TYPE:
            pose = None
            # TODO: how to get free position inside polygon? some perception node?
        else:
            return None
        return pose

    def manip_pick(self, instruction):
        """

        :type instruction: ProgramItem
        :return:
        """
        obj_id = self.get_pick_obj_id(instruction)

        if obj_id is None:
            return self.INST_BAD_DATA
        if self.pick_object(obj_id):
            self.holding_object = obj_id
            return self.INST_OK
        else:
            return self.INST_FAILED

    def manip_place(self, instruction):
        """

        :type instruction: ProgramItem
        :return:
        """

        pose = self.get_place_pose(instruction)

        if pose is None:
            return self.INST_BAD_DATA
        else:
            if self.place_object(self.holding_object, pose):
                self.holding_object = None
                return self.INST_OK
            else:
                return self.INST_FAILED

    def manip_pick_place(self, instruction):
        
        obj_id = self.get_pick_obj_id(instruction)
        pose = self.get_place_pose(instruction)
        self.send_feedback(obj_id) # TODO also publish selected place pose when not given (polygon)
        if obj_id is None or pose is None:
            return self.INST_BAD_DATA
        if self.pick_object(obj_id): # TODO call pick&place and not pick and then place
            self.holding_object = obj_id
            if self.place_object(obj_id, pose):
                self.holding_object = None
                return self.INST_OK
        return self.INST_FAILED

    def wait(self, instruction):
        """

        :type instruction: ProgramItem
        :return:
        """
        
        rate = rospy.Rate(10)

        if instruction.spec == instruction.WAIT_FOR_USER:
            while self.user_id == 0:
                rate.sleep()
        else:
            while self.user_id > 0:
                rate.sleep()
                
        return self.INST_OK

    def unknown_instruction(self, instruction):
        print "F*ck it all, i don't know this instruction!"
        return self.INST_FAILED

    def nop(self, instruction=None):
        return self.INST_OK

    def user_status_cb(self, data):
        """

        :type data: UserStatus
        :return:
        """
        self.user_id = data.user_id

        pass

    def selected_object_cb(self, data):
        """

        :type data: String
        :return:
        """
        self.selected_object = data.data
        self.selected_object_last_update = rospy.get_rostime()

    def selected_place_cb(self, data):
        """

        :type data: PoseStamped
        :return:
        """
        self.selected_place = data

    def objects_cb(self, objects_data):
        """

        :type objects_data: InstancesArray
        :return:
        """
        self.objects = objects_data

    def check_user_active(self):
        return self.user_id != 0

    def calibrate(self, action_name, server="unknown", timeout=5):
        client = actionlib.SimpleActionClient(action_name, LocalizeAgainstUMFAction)
        rospy.logdebug("Waiting for server (" + server + ")")
        client.wait_for_server()
        rospy.logdebug("Server ready (" + server + ")")
        goal = LocalizeAgainstUMFGoal()
        goal.timeout = rospy.Duration(timeout)
        rospy.logdebug("Sending goal to server (" + server + ")")
        client.send_goal(goal)
        rospy.logdebug("Waiting for results  (" + server + ")")
        client.wait_for_result()
        return not client.get_result().result
        pass

    def calibrate_all(self, table_calibration=True, pr2_calibration=True):
        rospy.loginfo("Starting calibrating process")
        rospy.logdebug("Waiting for service " + self.show_marker_service)
        rospy.wait_for_service(self.show_marker_service)
        try:
            show_marker = rospy.ServiceProxy(self.show_marker_service, Empty)
            show_marker()

        except rospy.ServiceException, e:
            rospy.logerr("Service call failed: %s"%e)
            return False

        while table_calibration:
            if self.calibrate(self.table_localize_action, "table", 5):
                table_calibration = False
                rospy.loginfo("Table successfully calibrated")
            else:
                rospy.logwarn("Table calibration failed! Trying every 5 sec")
                time.sleep(5)
        while pr2_calibration:
            if self.calibrate(self.pr2_localize_action, "pr2", 5):
                pr2_calibration = False
                rospy.loginfo("PR2 successfully calibrated")
            else:
                rospy.logwarn("PR2 calibration failed! Trying every 5 sec")
                time.sleep(5)
        rospy.loginfo("Calibration done, hiding umf marker")
        rospy.logdebug("Waiting for service " + self.hide_marker_service)
        rospy.wait_for_service(self.hide_marker_service)
        try:
            hide_marker = rospy.ServiceProxy(self.hide_marker_service, Empty)
            hide_marker()

        except rospy.ServiceException, e:
            rospy.logerr("Service call failed: %s"%e)
            return False

        pass

    def state_unknown(self):
        pass

    def state_start(self):
        rospy.loginfo('Starting')
        self.state = self.SYSTEM_CALIBRATING

    def state_calibrating(self):
        self.calibrate_all(self.calibrate_table, self.calibrate_pr2)
        self.state = self.SYSTEM_STARTING_PROGRAM_SERVER

    def state_switcher(self):
        states = {
            self.SYSTEM_START: self.state_start,
            self.SYSTEM_CALIBRATING: self.state_calibrating,
            self.SYSTEM_STARTING_PROGRAM_SERVER: self.state_starting_program_server,
            self.SYSTEM_READY_FOR_PROGRAM_REQUESTS: self.state_ready_for_program_requests,
            self.SYSTEM_STOPPING_PROGRAM_SERVER: self.state_stopping_program_server
        }
        return states.get(self.state, self.state_unknown)

    def state_starting_program_server(self):
        rospy.loginfo('Starting program server')
        self.prog_as = actionlib.SimpleActionServer("/art/brain/do_program", RobotProgramAction,
                                                    execute_cb=self.execute_cb, auto_start=False)
        self.prog_as.start()
        self.state = self.SYSTEM_READY_FOR_PROGRAM_REQUESTS

    def state_ready_for_program_requests(self):
        if self.stop_server:
            self.state = self.SYSTEM_STOPPING_PROGRAM_SERVER
            self.stop_server = False

    def state_stopping_program_server(self):
        self.prog_as = None
        if self.recalibrate:
            self.state = self.SYSTEM_CALIBRATING
            self.recalibrate = False
        else:
            self.quit = True

    def show_umf_marker(self):
        pass

    def hide_umf_marker(self):
        pass

    def pick_object(self, object_id):
        """

        :type object_id: str
        :return:
        """
        goal = pickplaceGoal()
        goal.id = object_id
        goal.operation = goal.PICK
        goal.keep_orientation = False
        rospy.loginfo("Picking object with ID: " + str(object_id))
        self.pp_client.send_goal(goal)
        self.pp_client.wait_for_result()
        # TODO: make some error msg etc
        '''rospy.loginfo('got result')
        print self.pp_client.get_result()
        print "status: " + self.pp_client.get_goal_status_text()
        print "state: " + str(self.pp_client.get_state())
        '''
        if self.pp_client.get_result().result == 0:
            return True
        else:
            return False

    def place_object(self, obj, place):
        """

        :type obj: str
        :type place: Pose
        :return:
        """
        goal = pickplaceGoal()
        goal.operation = goal.PLACE
        goal.id = obj
        goal.place_pose = PoseStamped()

        goal.place_pose = place
        goal.place_pose.header.stamp = rospy.Time.now()
        # TODO: how to deal with this?
        goal.place_pose.pose.position.z = 0.06# + obj.bbox.dimensions[2]/2
        self.pp_client.send_goal(goal)
        self.pp_client.wait_for_result()
        if self.pp_client.get_result().result == 0:
            return True
        else:
            return False

    def publish_state(self):
        data = SystemState()
        data.state = self.state
        self.state_publisher.publish(data)

    def process(self):
        state_function = self.state_switcher()
        state_function()
        self.publish_state()


if __name__ == '__main__':
    rospy.init_node('art_brain')
    try:
        node = ArtBrain()
        rate = rospy.Rate(30)
        while not rospy.is_shutdown() or node.quit:
            node.process()
            rate.sleep()
    except rospy.ROSInterruptException:
        pass
