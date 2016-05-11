#!/usr/bin/env python

import sys
import rospy
import actionlib
from art_msgs.msg import RobotProgramAction, RobotProgramFeedback,  RobotProgramResult
from art_msgs.srv import getProgram

prog_as = None

def execute_cb(goal):
    
    global prog_as
    
    for prog in goal.program_array.programs:
    
        for it in prog.items:
            
            feedback = RobotProgramFeedback()
            feedback.current_program = prog.id
            feedback.current_item = it.id
            prog_as.publish_feedback(feedback)
            
            rospy.loginfo('Program id: ' + str(prog.id) + ', item id: ' + str(it.id))
            
            rospy.sleep(1)
            
    res = RobotProgramResult()
    res.result = RobotProgramResult.SUCCESS
    prog_as.set_succeeded(res)

def main(args):
    
    global prog_as

    rospy.init_node('test_brain')
    
    prog_as = actionlib.SimpleActionServer("/art_brain/do_program", RobotProgramAction, execute_cb= execute_cb, auto_start = False)
    prog_as.start()
    
    rospy.spin()
    
if __name__ == '__main__':
    try:
        main(sys.argv)
    except KeyboardInterrupt:
        print("Shutting down")
