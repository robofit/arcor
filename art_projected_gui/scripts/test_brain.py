#!/usr/bin/env python

import sys
import rospy
from art_msgs.srv import ProgramIdTrigger, ProgramIdTriggerResponse
from art_msgs.msg import InterfaceState, ProgramItem, LearningRequestAction, LearningRequestFeedback, LearningRequestResult
from art_utils import InterfaceStateManager, ProgramHelper, ArtApiHelper
import actionlib
from std_srvs.srv import Trigger, TriggerResponse
from geometry_msgs.msg import PoseStamped
from copy import deepcopy

prog_timer = None
current_item = (0, 0)  # block_id, item_id
state_manager = None
ph = None
art = None
iters = 0
action_server = None
left_gripper_pub = None
right_gripper_pub = None
gripper_timer = None


def gripper_timer(event):

    global left_gripper_pub
    global right_gripper_pub

    ps = PoseStamped()
    ps.header.stamp = rospy.Time.now()
    ps.header.frame_id = "marker"
    ps.pose.position.x = 1.9
    ps.pose.position.y = 0.5
    ps.pose.position.z = 0.39
    ps.pose.orientation.w = 1.0

    left_gripper_pub.publish(deepcopy(ps))

    ps.pose.position.x = -0.5
    right_gripper_pub.publish(ps)


def timer_callback(event):

    global current_item
    global state_manager
    global ph
    global prog_timer
    global iters

    if current_item == (0, 0):

        prog_timer.shutdown()
        state_manager.set_system_state(InterfaceState.STATE_PROGRAM_FINISHED)
        state_manager.send()
        rospy.loginfo("Program finished")
        return

    flags = {}
    it = ph.get_item_msg(*current_item)
    if it.type == ProgramItem.PICK_FROM_POLYGON or it.type == ProgramItem.PLACE_TO_POSE:
        flags["SELECTED_OBJECT_ID"] = "my_object"

    state_manager.update_program_item(ph.get_program().header.id, current_item[
                                      0], ph.get_item_msg(*current_item), flags)

    if iters < 1:
        current_item = ph.get_id_on_success(*current_item)
    else:
        current_item = ph.get_id_on_failure(*current_item)

    if current_item == ph.get_first_item_id():

        iters += 1


def start_program(req):

    global prog_timer
    global current_item
    global state_manager
    global ph
    global art
    global iters

    program = art.load_program(req.program_id)

    if program is None:
        return ProgramIdTriggerResponse(success=False)

    if not ph.load(program):
        return ProgramIdTriggerResponse(success=False)

    current_item = ph.get_first_item_id()
    iters = 0

    state_manager.set_system_state(InterfaceState.STATE_PROGRAM_RUNNING)
    prog_timer = rospy.Timer(rospy.Duration(4), timer_callback)
    return ProgramIdTriggerResponse(success=True)

# def stop_program(req):
#
#    global prog_timer
#    prog_timer.shutdown()
#    # TODO state_manager.set_syst_state()
#    return stopProgramResponse(success=True)


def callback(old_state, new_state, flags):

    print "Got state update"


def learning_request_cb(goal):

    global action_server

    print "Received action goal"
    print goal

    fb = LearningRequestFeedback()

    for i in range(0, 11):

        fb.progress = i * 10
        action_server.publish_feedback(fb)
        rospy.sleep(0.1)

    res = LearningRequestResult()
    res.success = True

    action_server.set_succeeded(res)


def learning_srv_cb(req):

    global state_manager
    global art
    global ph

    program = art.load_program(req.program_id)

    if program is None:
        return ProgramIdTriggerResponse(success=False)

    if not ph.load(program):
        return ProgramIdTriggerResponse(success=False)

    current_item = ph.get_first_item_id()

    state_manager.update_program_item(req.program_id, current_item[0], ph.get_item_msg(*current_item), auto_send=False)
    state_manager.set_system_state(InterfaceState.STATE_LEARNING)

    rospy.loginfo("learning_srv_cb")
    return ProgramIdTriggerResponse(success=True)


def main(args):

    global state_manager
    global ph
    global art
    global action_server
    global left_gripper_pub
    global right_gripper_pub
    global gripper_timer

    rospy.init_node('test_brain')

    state_manager = InterfaceStateManager(InterfaceState.BRAIN_ID, callback)
    ph = ProgramHelper()

    state_manager.set_system_state(InterfaceState.STATE_IDLE)

    rospy.Service('/art/brain/program/start', ProgramIdTrigger, start_program)
    rospy.Service('/art/brain/learning/start', ProgramIdTrigger, learning_srv_cb)
    rospy.Service('/art/brain/learning/stop', Trigger, learning_srv_cb)
    # rospy.Service('/art/brain/program/stop',  stopProgram, stop_program)

    art = ArtApiHelper()

    action_server = actionlib.SimpleActionServer(
        '/art/brain/learning_request', LearningRequestAction, execute_cb=learning_request_cb, auto_start=False)
    action_server.start()

    left_gripper_pub = rospy.Publisher('/art/pr2/left_arm/gripper/pose', PoseStamped, queue_size=10)
    right_gripper_pub = rospy.Publisher('/art/pr2/right_arm/gripper/pose', PoseStamped, queue_size=10)

    gripper_timer = rospy.Timer(rospy.Duration(0.5), gripper_timer)

    rospy.spin()


if __name__ == '__main__':
    try:
        main(sys.argv)
    except KeyboardInterrupt:
        print("Shutting down")
