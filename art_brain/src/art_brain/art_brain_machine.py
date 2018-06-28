# from transitions import Machine
# from transitions.extensions import GraphMachine as Machine
from transitions.extensions import LockedMachine as Machine
from transitions import State


class ArtBrainMachine(object):
    states = [State(name='pre_init', on_enter=[], on_exit=[]),
              State(name='init', on_enter=['state_init_ros'], on_exit=[]),
              State(name='shutdown', on_enter=['state_shutdown'], on_exit=[]),
              State(name='waiting_for_action', on_enter=[
                    'state_waiting_for_action'], on_exit=[]),
              State(name='program_init', on_enter=[
                    'state_program_init'], on_exit=[]),
              State(name='program_run', on_enter=[
                    'state_program_run'], on_exit=[]),
              State(name='program_paused', on_enter=[
                    'state_program_paused'], on_exit=[]),


              State(name='program_error', on_enter=[
                    'state_program_error'], on_exit=[]),
              State(name='program_finished', on_enter=[
                    'state_program_finished'], on_exit=[]),
              State(name='program_load_instruction', on_enter=[
                    'state_program_load_instruction'], on_exit=[]),

              # learning
              State(name='learning_init', on_enter=[
                    'learning_load_block_id', 'state_learning_init'], on_exit=[]),
              State(name='learning_run', on_enter=[
                    'learning_load_block_id', 'state_learning_run'], on_exit=[]),

              # learning drilling and welding
              State(name='learning_welding_point', on_enter=[
                  'learning_load_block_id', 'state_learning_welding_point'], on_exit=[]),
              State(name='learning_welding_point_run', on_enter=[
                  'learning_load_block_id', 'state_learning_welding_point_run'], on_exit=[]),

              State(name='learning_welding_seam', on_enter=[
                  'learning_load_block_id', 'state_learning_welding_seam'], on_exit=[]),
              State(name='learning_welding_seam_run', on_enter=[
                  'learning_load_block_id', 'state_learning_welding_seam_run'], on_exit=[]),


              # learning others

              State(name='learning_wait', on_enter=[
                  'learning_load_block_id', 'state_learning_wait'], on_exit=[]),
              State(name='learning_step_done', on_enter=[
                  'learning_load_block_id', 'state_learning_step_done'], on_exit=[]),
              State(name='learning_step_error', on_enter=[
                  'learning_load_block_id', 'state_learning_step_error'], on_exit=[]),
              State(name='learning_done', on_enter=[
                  'learning_load_block_id', 'state_learning_done'], on_exit=[]),

              # visualize

              State(name='visualize_init', on_enter=[
                  'visualize_load_block_id', 'state_visualize_init'], on_exit=[]),
              State(name='visualize_run', on_enter=[
                  'visualize_load_block_id', 'state_visualize_run'], on_exit=[]),
              State(name='visualize_done', on_enter=[
                  'state_visualize_done'], on_exit=[])]

    def __init__(self, states, transitions):
        self.name = 'brain'
        self.states += states
        self.machine = Machine(model=self, states=ArtBrainMachine.states, initial='pre_init',
                               auto_transitions=False, send_event=True, queued=True)

        # *** transitions ***

        self.machine.add_transition('init', 'pre_init', 'init')
        self.machine.add_transition(
            'init_done', 'init', 'waiting_for_action', conditions='is_everything_calibrated')
        self.machine.add_transition(
            'program_start', 'waiting_for_action', 'program_init')
        self.machine.add_transition(
            'learning_start', 'waiting_for_action', 'learning_init')
        self.machine.add_transition(
            'visualize_start', 'waiting_for_action', 'visualize_init')

        # program
        self.machine.add_transition(
            'program_init_done', 'program_init', 'program_run')
        self.machine.add_transition('error', 'program_init', 'program_error')
        self.machine.add_transition('error', 'program_run', 'program_error')
        self.machine.add_transition(
            'program_error_handled', 'program_error', 'program_load_instruction')
        self.machine.add_transition(
            'program_error_shutdown', 'program_error', 'shutdown')
        self.machine.add_transition(
            'program_error_fatal', 'program_error', 'program_finished')
        self.machine.add_transition(
            'try_again', 'program_error', 'program_run')
        self.machine.add_transition(
            'skip', 'program_error', 'program_load_instruction')
        self.machine.add_transition(
            'fail', 'program_error', 'program_load_instruction')
        self.machine.add_transition(
            'done', 'program_load_instruction', 'program_run')
        self.machine.add_transition(
            'error', 'program_load_instruction', 'program_error')
        self.machine.add_transition(
            'finished', 'program_load_instruction', 'program_finished')
        self.machine.add_transition(
            'done', 'program_finished', 'waiting_for_action')
        self.machine.add_transition(
            'finished', 'program_run', 'program_finished')
        self.machine.add_transition(
            'pause', 'program_load_instruction', 'program_paused')
        self.machine.add_transition(
            'resume', 'program_paused', 'program_run')

        # learning

        self.machine.add_transition(
            'init_done', 'learning_init', 'learning_run')
        self.machine.add_transition(
            'done', 'learning_step_done', 'learning_run')
        self.machine.add_transition(
            'error_handled', 'learning_step_error', 'learning_run')
        self.machine.add_transition(
            'error_fatal', 'learning_step_error', 'waiting_for_action')
        self.machine.add_transition(
            'done', 'learning_done', 'waiting_for_action')
        self.machine.add_transition(
            'learning_done', 'learning_run', 'learning_done')

        for transition in transitions:
            self.machine.add_transition(transition[0], transition[1], transition[2])
