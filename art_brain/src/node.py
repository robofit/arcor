#!/usr/bin/env python

import rospy
import time

import actionlib
from art_msgs.msg import LocalizeAgainstUMFAction, LocalizeAgainstUMFGoal, LocalizeAgainstUMFResult
from std_srvs.srv import Empty, EmptyRequest, EmptyResponse
from art_msgs.msg import UserStatus,  UserActivity, InterfaceState,  InterfaceStateItem
from art_msgs.srv import startProgram,  startProgramResponse,  getProgram
from geometry_msgs.msg import PoseStamped, Pose
from std_msgs.msg import String
from art_msgs.msg import pickplaceAction, pickplaceGoal, SystemState, ObjInstance, InstancesArray, ProgramItem
import matplotlib.path as mplPath
import numpy as np
import random
from art_interface_utils.interface_state_manager import interface_state_manager

class ArtBrain:
    UNKNOWN = -2  # should not happen!
    NOP = -1  # no operation
    GET_READY = 0  # retract arms etc.
    MANIP_PICK = 1
    MANIP_PLACE = 2 # TODO proc nepouzit ProgramItem.MANIP_PLACE?
    MANIP_PICK_PLACE = 3
    WAIT = 4

    INST_OK = 100
    INST_BAD_DATA = 101
    INST_FAILED = 102

    SYSTEM_UNKNOWN = 0 # TODO proc nepouzit SystemState.SYSTEM_UNKNOWN?
    SYSTEM_START = 1
    SYSTEM_CALIBRATING = 2
    SYSTEM_STARTING_PROGRAM_SERVER = 3
    SYSTEM_READY_FOR_PROGRAM_REQUESTS = 4
    SYSTEM_STOPPING_PROGRAM_SERVER = 5


    def __init__(self):
        self.show_marker_service = rospy.get_param('show_marker_service', '/art/interface/projected_gui/show_marker')
        self.hide_marker_service = rospy.get_param('hide_marker_service', '/art/interface/projected_gui/hide_marker')
        self.table_localize_action = rospy.get_param('table_localize_action', '/umf_localizer_node_table/localize')
        self.pr2_localize_action = rospy.get_param('pr2_localize_action', '/umf_localizer_node/localize')

        self.calibrate_pr2 = rospy.get_param('calibrate_pr2', False)
        self.calibrate_table = rospy.get_param('calibrate_table', False)

        self.user_status_sub = rospy.Subscriber("/art/user/status", UserStatus, self.user_status_cb)
        self.user_activity_sub = rospy.Subscriber("/art/user/activity", UserActivity, self.user_activity_cb)
        
        self.srv_program_start = rospy.Service('/art/brain/program/start', startProgram, self.program_start_cb)
        self.srv_program_stop = rospy.Service('/art/brain/program/stop', Empty, self.program_stop_cb)
        #self.srv_program_pause = rospy.Service(/art/brain/program/pause', Empty, self.program_pause_cb)
        #self.srv_program_resume = rospy.Service(/art/brain/program/resume', Empty, self.program_resume_cb)
        
        self.state_manager = interface_state_manager(InterfaceState.BRAIN_ID)
        
        self.user_activity = None
        
        self.objects_sub = rospy.Subscriber("/art/object_detector/object_filtered", InstancesArray, self.objects_cb)

        self.state_publisher = rospy.Publisher("/art/brain/system_state", SystemState, queue_size=1)

        self.pp_client = actionlib.SimpleActionClient('/pr2_pick_place_left/pp', pickplaceAction)

        self.state = self.SYSTEM_START
        self.user_id = 0
      
        self.objects = InstancesArray()
        self.executing_program = False

        self.instruction = None
        self.holding_object = None
        self.stop_server = False
        self.recalibrate = False

        self.quit = False
        
        self.program = None
        self.prog_id = None
        self.it_id = None
        
    def program_start_cb(self,  req):
        
        resp = startProgramResponse()
        
        if self.executing_program:
            
            resp.success = False
            resp.error = 'Program already running'
            return resp
        
        rospy.loginfo('Loading program ' + str(req.program_id) + ' from db...')
        
        prog_srv = rospy.ServiceProxy('/art/db/program/get', getProgram)
        
        try:
            presp = prog_srv(req.program_id)
        except rospy.ServiceException, e:
            rospy.logerr('Cannot get program :(')
            resp.success = False
            resp.error = 'Cannot get program'
            return resp
            
        self.prog_id = req.program_id
        self.program = presp.program
        
        rospy.loginfo('Starting program')
        self.executing_program = True
        resp.success = True
        return resp

    def program_stop_cb(self,  req):
        
        rospy.loginfo('Stopping program ' + str(req.program_id) + '...')
        self.executing_program = False
        return EmptyResponse()

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
        print "get_item_by_id: " + str(item_id)
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

            for point in instruction.pick_polygon.polygon.points: # TODO check frame_id and transform to table frame?
                pick_polygon.append([point.x,  point.y])
            if len(pick_polygon) > 0:
                pol = mplPath.Path(np.array(pick_polygon),  closed = True)

            # shuffle the array to not get the same object each time
            #random.shuffle(self.objects.instances)
            
            print self.objects.instances
            
            for obj in self.objects.instances:
                
                if pol is None:
                    
                    print "no polygon"
                    
                    # if no pick polygon is specified - let's take the first object of that type
                    if obj.object_type == instruction.object:
                        obj_id = obj.object_id
                        break
                        
                else:
                    
                    print "polygon"
                    
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
            print "strange instruction.spec: " + str(instruction.spec)
            return self.INST_BAD_DATA
        return obj_id

    def get_place_pose(self, instruction):
        #if self.holding_object is None:
        #    return None
        if instruction.spec == instruction.MANIP_ID:
            pose = instruction.place_pose
        elif instruction.spec == instruction.MANIP_TYPE:
            #pose = None
            pose = instruction.place_pose
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
        
        print "manip_pick_place"
        
        obj_id = self.get_pick_obj_id(instruction)
        pose = self.get_place_pose(instruction)
        
        self.state_manager.select_object_id(obj_id)
        # TODO also publish selected place pose when not given (polygon)
        
        if obj_id is None or pose is None:
            print 'could not get obj_id or pose'
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
        print "waiting"
        
        #return self.INST_OK
        
        rate = rospy.Rate(10)

        if instruction.spec == instruction.WAIT_FOR_USER:
            while self.user_activity != UserActivity.READY:
                rate.sleep()
        elif instruction.spec == instruction.WAIT_UNTIL_USER_FINISHES:
            while self.user_activity != UserActivity.WORKING:
                rate.sleep()
        else:
            return self.INST_BAD_DATA
                
        return self.INST_OK

    def unknown_instruction(self, instruction):
        print "F*ck it all, i don't know this instruction!"
        return self.INST_FAILED

    def nop(self, instruction=None):
        return self.INST_OK

    def user_activity_cb(self,  data):
        
        self.user_activity = data.activity

    def user_status_cb(self, data):
        """

        :type data: UserStatus
        :return:
        """
        self.user_id = data.user_id

        pass

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
        #self.state = self.SYSTEM_STARTING_PROGRAM_SERVER

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
        #self.prog_as = actionlib.SimpleActionServer("/art/brain/do_program", RobotProgramAction,
        #                                            execute_cb=self.execute_cb, auto_start=False)
        #self.prog_as.start()
        self.state = self.SYSTEM_READY_FOR_PROGRAM_REQUESTS

    def state_ready_for_program_requests(self):
        if self.stop_server:
            self.state = self.SYSTEM_STOPPING_PROGRAM_SERVER
            self.stop_server = False # TODO refuse requests to startProgram service when 'server' is not enabled?
            
        if self.executing_program: # flag set in service request
            
            # for it in prog.items:
            it = self.program.items[0]
            while self.executing_program:
                
                self.prog_id = self.program.id
                self.it_id = it.id
                
                self.state_manager.set_syst_state(InterfaceStateItem.STATE_PROGRAM_RUNNING,  self.prog_id,  self.it_id)
                self.state_manager.clear_all()
                
                # let's tell interface what to display
                if it.type == ProgramItem.MANIP_PICK_PLACE:
            
                    if it.spec == ProgramItem.MANIP_ID:
                        
                        self.state_manager.select_object_id(it.object)
                        
                    #elif it.spec == ProgramItem.MANIP_TYPE:
                    #    self.state_manager.publish(InterfaceState.EVT_OBJECT_TYPE_SELECTED,  obj)
                        
                    if len(it.pick_polygon.polygon.points) > 0: self.state_manager.select_polygon(it.pick_polygon)
                    
                    self.state_manager.select_place(it.place_pose)
                    
                else:

                    # TODO other types of operations
                    pass
                
                rospy.loginfo('Program id: ' + str(self.program.id) + ', item id: ' + str(it.id) + ', item type: ' + str(it.type))

                self.instruction = it.type
                instruction_function = self.instruction_switcher()
                result = instruction_function(it)
                if result == self.INST_OK:
                    it = self.get_item_by_id(self.program, it.on_success)
                elif result == self.INST_BAD_DATA or result == self.INST_FAILED:
                    it = self.get_item_by_id(self.program, it.on_failure)
                    print "bad_data/failed"
                else:
                    it = None
                    print "strange return value"

                if it is None:
                    rospy.logerr('Error during instruction evaluation')
                    # TODO feedback
                    self.executing_program = False
                    return
                    
            if not self.executing_program:
                
                pass
                #self.state_manager.publish(InterfaceState.EVT_STATE_PROGRAM_STOPPED)

        # TODO feedback
        self.executing_program = False

    def state_stopping_program_server(self):
        #self.prog_as = None
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
    
    rospy.loginfo('Waiting for other nodes to come up...')
    rospy.wait_for_service('/art/db/program/get')
    rospy.wait_for_service('/art/db/program/store')
    rospy.loginfo('Ready!')
    
    try:
        node = ArtBrain()
        rate = rospy.Rate(30)
        while not rospy.is_shutdown() or node.quit:
            node.process()
            rate.sleep()
    except rospy.ROSInterruptException:
        pass
