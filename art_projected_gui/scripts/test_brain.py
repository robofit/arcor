#!/usr/bin/env python

import sys
import rospy
from art_msgs.srv import getProgram,  startProgram,  startProgramResponse
from art_msgs.msg import InterfaceState,  InterfaceStateItem
from art_interface_utils.interface_state_manager import interface_state_manager

prog_timer = None
program = None
current_item = 0 # idx of item
state_manager = None

def timer_callback(event):

    global current_item
    global program
    global state_manager

    state_manager.set_syst_state(InterfaceStateItem.STATE_PROGRAM_RUNNING,  program.id,  program.items[current_item].id)
    state_manager.clear_all()

    # TODO set_object / place etc.

    current_item += 1

    if current_item == len(program.items): current_item=0

def start_program(req):

    global prog_timer
    global program
    global current_item

    prog_srv = rospy.ServiceProxy('/art/db/program/get', getProgram)

    try:
        resp = prog_srv(req.program_id)
        program = resp.program
    except rospy.ServiceException, e:
        print "Service call failed: %s"%e
        program = None
        return startProgramResponse(success=False)

    current_item = 0

    prog_timer = rospy.Timer(rospy.Duration(2), timer_callback)
    return startProgramResponse(success=True)

#def stop_program(req):
#
#    global prog_timer
#    prog_timer.shutdown()
#    # TODO state_manager.set_syst_state()
#    return stopProgramResponse(success=True)

def main(args):

    global state_manager

    rospy.init_node('test_brain')

    state_manager = interface_state_manager(InterfaceState.BRAIN_ID)

    rospy.Service('/art/brain/program/start',  startProgram, start_program)
    #rospy.Service('/art/brain/program/stop',  stopProgram, stop_program)

    rospy.spin()

if __name__ == '__main__':
    try:
        main(sys.argv)
    except KeyboardInterrupt:
        print("Shutting down")
