#!/usr/bin/env python

import sys
import rospy
from art_msgs.srv import getProgram,  startProgram,  startProgramResponse
from art_msgs.msg import InterfaceState,  ProgramItem
from art_utils.interface_state_manager import InterfaceStateManager
prog_timer = None
program = None
current_item = 0  # idx of item
state_manager = None


def timer_callback(event):

    global current_item
    global program
    global state_manager

    flags = {}
    if program.blocks[0].items[current_item].type == ProgramItem.MANIP_PICK_PLACE:
        flags["SELECTED_OBJECT_ID"] = "my_object"

    state_manager.update_program_item(program.id,  program.blocks[0].items[current_item],  flags)

    # TODO go through blocks/items according to their ids
    current_item += 1

    if current_item == len(program.blocks[0].items):

        current_item = 0


def start_program(req):

    global prog_timer
    global program
    global current_item
    global state_manager

    prog_srv = rospy.ServiceProxy('/art/db/program/get', getProgram)

    try:
        resp = prog_srv(req.program_id)
        program = resp.program
    except rospy.ServiceException, e:
        print "Service call failed: " + str(e)
        program = None
        return startProgramResponse(success=False)

    current_item = 0

    state_manager.set_system_state(InterfaceState.STATE_PROGRAM_RUNNING)
    prog_timer = rospy.Timer(rospy.Duration(4), timer_callback)
    return startProgramResponse(success=True)

# def stop_program(req):
#
#    global prog_timer
#    prog_timer.shutdown()
#    # TODO state_manager.set_syst_state()
#    return stopProgramResponse(success=True)


def callback(old_state,  new_state,  flags):

    print "Got state update"


def main(args):

    global state_manager

    rospy.init_node('test_brain')

    state_manager = InterfaceStateManager(InterfaceState.BRAIN_ID,  callback)

    rospy.Service('/art/brain/program/start',  startProgram, start_program)
    # rospy.Service('/art/brain/program/stop',  stopProgram, stop_program)

    rospy.spin()

if __name__ == '__main__':
    try:
        main(sys.argv)
    except KeyboardInterrupt:
        print("Shutting down")
