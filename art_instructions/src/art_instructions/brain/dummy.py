from art_instructions.brain import BrainFSM, BrainInstruction
from transitions import State
import rospy


class DummyFSM(BrainFSM):
    states = [
        State(name='dummy', on_enter=[
            'state_update_program_item', 'state_dummy']),
        State(name='learning_dummy', on_enter=[
            'learning_load_block_id', 'state_learning_dummy']),
        State(name='learning_dummy_run', on_enter=[
            'learning_load_block_id', 'state_learning_dummy_run']),

    ]

    transitions = [
        ('dummy', 'program_run', 'dummy'),
        ('done', 'dummy', 'program_load_instruction'),
        ('error', 'dummy', 'program_error'),
        ('dummy', 'learning_run', 'learning_dummy'),
        ('done', 'learning_dummy', 'learning_step_done'),
        ('error', 'learning_dummy', 'learning_step_error'),
        ('dummy_run', 'learning_run', 'learning_dummy_run'),
        ('done', 'learning_dummy_run', 'learning_run'),
        ('error', 'learning_dummy_run', 'learning_step_error')

    ]

    state_functions = [
        'state_dummy',
        'state_learning_dummy',
        'state_learning_dummy_run'
    ]

    def run(self):
        self.fsm.dummy()

    def learning(self):
        self.fsm.dummy()

    def learning_run(self):
        self.fsm.dummy_run()

    def state_dummy(self, event):
        rospy.logdebug('Current state: state_dummy')
        self.fsm.done(success=True)

    def state_learning_dummy(self, event):
        rospy.logdebug('Current state: state_dummy_points')

    def state_learning_dummy_run(self, event):
        rospy.logdebug('Current state: state_learning_dummy_run')
        self.fsm.done(success=True)
