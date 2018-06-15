from art_instructions.brain import BrainFSM, BrainInstruction
from transitions import State
import rospy


class VisualInspection(BrainInstruction):

    def __init__(self, *args, **kwargs):

        super(VisualInspection, self).__init__(*args, **kwargs)


class VisualInspectionLearn(VisualInspection):

    def __init__(self, *args, **kwargs):

        super(VisualInspectionLearn, self).__init__(*args, **kwargs)


class VisualInspectionRun(VisualInspection):

    def __init__(self, *args, **kwargs):

        super(VisualInspectionRun, self).__init__(*args, **kwargs)


class VisualInspectionFSM(BrainFSM):
    states = [
        State(name='visual_inspection', on_enter=[
            'state_update_program_item', 'check_robot_in', 'state_visual_inspection'],
              on_exit=['check_robot_out']),
        State(name='learning_visual_inspection', on_enter=[
            'check_robot_in', 'learning_load_block_id', 'state_learning_visual_inspection_run'],
              on_exit=['check_robot_out']),
        State(name='learning_visual_inspection', on_enter=[
            'check_robot_in', 'learning_load_block_id', 'state_learning_visual_inspection'],
              on_exit=['check_robot_out', 'state_visual_inspection_exit'])
    ]

    transitions = [
        ('visual_inspection', 'program_run', 'visual_inspection'),
        ('done', 'visual_inspection', 'program_load_instruction'),
        ('error', 'visual_inspection', 'program_error'),
        ('visual_inspection', 'learning_run', 'learning_visual_inspection'),
        ('done', 'learning_visual_inspection', 'learning_step_done'),
        ('error', 'learning_visual_inspection', 'learning_step_error'),
        ('visual_inspection_run', 'learning_run', 'learning_visual_inspection_run'),
        ('done', 'learning_visual_inspection_run', 'learning_run'),
        ('error', 'learning_visual_inspection_run', 'learning_step_error')
    ]

    state_functions = [
        'state_visual_inspection',
        'state_learning_visual_inspection_run',
        'state_learning_visual_inspection'
    ]

    def __init__(self, *args, **kwargs):

        super(VisualInspectionFSM, self).__init__(*args, **kwargs)

    def run(self):
        self.fsm.visual_inspection()

    def learning(self):
        self.fsm.visual_inspection()

    def learning_run(self):
        self.fsm.visual_inspection_run()

    def state_visual_inspection(self):
        rospy.logdebug('Current state: state_visual_inspection')

    def state_learning_visual_inspection_run(self):
        rospy.logdebug('Current state: state_learning_visual_inspection_run')

    def state_learning_visual_inspection(self):
        rospy.logdebug('Current state: state_learning_visual_inspection')