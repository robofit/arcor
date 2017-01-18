#!/usr/bin/env python

import rospy
import time

import actionlib
from art_msgs.msg import LocalizeAgainstUMFAction, LocalizeAgainstUMFGoal, LocalizeAgainstUMFResult
from std_srvs.srv import Empty, EmptyRequest, EmptyResponse
from art_msgs.msg import UserStatus,  UserActivity, InterfaceState
from art_msgs.srv import startProgram,  startProgramResponse,  getProgram
from geometry_msgs.msg import PoseStamped, Pose
from std_msgs.msg import String
from art_msgs.msg import pickplaceAction, pickplaceGoal, SystemState, ObjInstance, InstancesArray, ProgramItem
import matplotlib.path as mplPath
import numpy as np
import random
from art_utils import InterfaceStateManager,  ArtApiHelper,  ProgramHelper

#from transitions import Machine
from transitions.extensions import GraphMachine as Machine
from transitions import State


class ArtBrain(object):
    
    states = [State(name='pre_init', on_enter=[],  on_exit=[]), 
                    State(name='init', on_enter=['init_ros_cb'],  on_exit=[]), 
                    State(name='program_init', on_enter=['program_init_cb'],  on_exit=[]), 
                    State(name='program_run', on_enter=['program_run_cb'],  on_exit=[]), 
                    State(name='pick', on_enter=[],  on_exit=[]), 
                    State(name='place', on_enter=[],  on_exit=[]), 
                    State(name='pick_place',   on_enter=[],  on_exit=[]), 
                    State(name='wait', on_enter=[],  on_exit=[]), 
                    State(name='program_error', on_enter=[],  on_exit=[]), 
                    State(name='teaching_init', on_enter=[],  on_exit=[])]
    
    def __init__(self):
        self.name = 'brain'
        
        self.machine = Machine(model=self,  states=ArtBrain.states, initial='pre_init',  auto_transitions=False, send_event=True)
    
        # *** transitions ***
        
        self.machine.add_transition('init',  'pre_init',  'init')
        
        # program
        self.machine.add_transition('program_start',  'init',  'program_init')
        self.machine.add_transition('program_init_done',  'program_init',  'program_run')
        self.machine.add_transition('failure',  'program_init',  'program_error')
        self.machine.add_transition('failure',  'program_run',  'program_error')
        self.machine.add_transition('program_error_handled', 'program_error',  'init')
                
        # pick instruction
        self.machine.add_transition('pick',  'program_run',  'pick')
        self.machine.add_transition('success',  'pick',  'program_run')
        self.machine.add_transition('failuer',  'pick',  'program_error')
        
        # place instruction
        self.machine.add_transition('place',  'program_run',  'place')
        self.machine.add_transition('success',  'place',  'program_run')
        self.machine.add_transition('failuer',  'place',  'program_error')
        
        # pick_place instruction
        self.machine.add_transition('pick_place',  'program_run',  'pick_place')
        self.machine.add_transition('success',  'pick_place',  'program_run')
        self.machine.add_transition('failuer',  'pick_place',  'program_error')
        
        # wait instruction
        self.machine.add_transition('wait',  'program_run',  'wait')
        self.machine.add_transition('success',  'wait',  'program_run')
        self.machine.add_transition('failuer',  'wait',  'program_error')
        
        # self.machine.graph.draw('my_state_diagram.png', prog='dot')
        
        self.init()
        
    def init_ros_cb(self,  event):
        rospy.loginfo('Waiting for other nodes to come up...')
        rospy.wait_for_service('/art/db/program/get')
        rospy.wait_for_service('/art/db/program/store')
        rospy.loginfo('Ready, waiting for program')
        self.program_start()

    def program_init_cb(self,  event):
        rospy.loginfo('New program ready!')
        self.program_init_done()
        
    def program_run_cb(self,  event):
        rospy.loginfo('Program running')
        
    def pick_cb(self,  event):
        pass
        
    def place_cb(self,  event):
        pass
        
    def pickplace_cb(self,  event):
        pass
    

if __name__ == '__main__':
    rospy.init_node('new_art_brain')
  
    try:
        node = ArtBrain()
        
        rate = rospy.Rate(30)
        while not rospy.is_shutdown():
            # node.process()
            rate.sleep()
    except rospy.ROSInterruptException:
        pass
