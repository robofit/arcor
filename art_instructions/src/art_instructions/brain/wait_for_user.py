from art_instructions.brain import BrainFSM, BrainInstruction
from transitions import State
import rospy
from art_msgs.msg import UserActivity


class WaitForUser(BrainInstruction):
    pass


class WaitForUserLearn(WaitForUser):
    pass


class WaitForUserRun(WaitForUser):
    pass


class WaitForUserFSM(BrainFSM):
    states = [
        State(name='wait_for_user', on_enter=[
            'state_update_program_item', 'check_robot_in', 'state_wait_for_user'], on_exit=['check_robot_out']),

    ]

    transitions = [
        ('wait_for_user', 'program_run', 'wait_for_user'),
        ('done', 'wait_for_user', 'program_load_instruction'),
        ('error', 'wait_for_user', 'program_error'),
    ]

    state_functions = [
        'state_wait_for_user'
    ]

    def run(self):
        self.fsm.wait_for_user()

    def state_wait_for_user(self, event):
        rospy.logdebug('Current state: state_wait_for_user')

        self.brain.state_manager.update_program_item(
            self.brain.ph.get_program_id(), self.brain.block_id, self.brain.instruction)

        rate = rospy.Rate(10)

        while self.brain.user_activity != UserActivity.READY and self.brain.executing_program \
                and not rospy.is_shutdown():
            rate.sleep()

        self.fsm.done(success=True)
