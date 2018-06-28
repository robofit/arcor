from art_instructions.brain import BrainFSM, BrainInstruction
from transitions import State
import rospy


class GetReady(BrainInstruction):
    pass


class GetReadyLearn(GetReady):
    pass


class GetReadyRun(GetReady):
    pass


class GetReadyFSM(BrainFSM):
    states = [
        State(name='get_ready', on_enter=[
            'state_update_program_item', 'check_robot_in', 'state_get_ready'], on_exit=['check_robot_out']),
        State(name='learning_get_ready_run', on_enter=[
            'check_robot_in', 'learning_load_block_id', 'state_learning_get_ready_run'],
            on_exit=['check_robot_out'])
    ]

    transitions = [
        ('get_ready', 'program_run', 'get_ready'),
        ('done', 'get_ready', 'program_load_instruction'),
        ('error', 'get_ready', 'program_error'),
        ('get_ready_run', 'learning_run', 'learning_get_ready_run'),
        ('done', 'learning_get_ready_run', 'learning_run'),
        ('error', 'learning_get_ready_run', 'learning_step_error')
    ]

    state_functions = [
        'state_get_ready',
        'state_learning_get_ready_run'
    ]

    def run(self):
        self.fsm.get_ready()

    def learning_run(self):
        self.fsm.get_ready_run()

    def state_get_ready(self, event):
        rospy.logdebug('Current state: state_get_ready')
        if not self.brain.check_robot():
            return
        self.brain.state_manager.update_program_item(
            self.brain.ph.get_program_id(), self.brain.block_id, self.brain.instruction)
        # TODO: call some service to set PR2 to ready position
        # TODO handle if it fails
        severity, error, arm_id = self.brain.robot.arms_get_ready()
        if error is not None:
            rospy.logerr("Error while geting ready: ", arm_id)
            self.fsm.error(severity=severity, error=error)
        else:
            self.fsm.done(success=True)

    def state_learning_get_ready_run(self, event):
        rospy.logdebug('Current state: state_get_ready')
        if not self.brain.check_robot():
            return
        self.brain.state_manager.update_program_item(
            self.brain.ph.get_program_id(), self.brain.block_id, self.brain.instruction)
        # TODO: call some service to set PR2 to ready position
        # TODO handle if it fails
        severity, error, arm_id = self.brain.robot.arms_get_ready()
        if error is not None:
            rospy.logerr("Error while geting ready: ", arm_id)
            self.fsm.error(severity=severity, error=error)
        else:
            self.fsm.done(success=True)
