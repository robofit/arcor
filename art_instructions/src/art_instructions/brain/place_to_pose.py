from art_instructions.brain import BrainFSM, BrainInstruction
from transitions import State
import rospy
from art_brain import ArtBrainErrors, ArtBrainErrorSeverities
from art_msgs.msg import ProgramItem, InterfaceState


class PlaceToPose(BrainInstruction):
    pass


class PlaceToPoseLearn(PlaceToPose):
    pass


class PlaceToPoseRun(PlaceToPose):
    pass


class PlaceToPoseFSM(BrainFSM):
    states = [
        State(name='place_to_pose', on_enter=[
            'state_update_program_item', 'check_robot_in', 'state_place_to_pose'],
            on_exit=['check_robot_out']),
        State(name='learning_place_to_pose', on_enter=[
            'learning_load_block_id', 'state_learning_place_to_pose'], on_exit=[]),
        State(name='learning_place_to_pose_run', on_enter=[
            'check_robot_in', 'learning_load_block_id', 'state_learning_place_to_pose_run'],
            on_exit=['check_robot_out']),
    ]

    transitions = [
        ('place_to_pose', 'program_run', 'place_to_pose'),
        ('done', 'place_to_pose', 'program_load_instruction'),
        ('error', 'place_to_pose', 'program_error'),
        ('place_to_pose', 'learning_run', 'learning_place_to_pose'),
        ('done', 'learning_place_to_pose', 'learning_step_done'),
        ('error', 'learning_place_to_pose', 'learning_step_error'),
        ('place_to_pose_run', 'learning_run', 'learning_place_to_pose_run'),
        ('done', 'learning_place_to_pose_run', 'learning_run'),
        ('error', 'learning_place_to_pose_run', 'learning_step_error')
    ]

    state_functions = [
        'state_place_to_pose',
        'state_learning_place_to_pose',
        'state_learning_place_to_pose_run'
    ]

    def run(self):
        self.fsm.place_to_pose()

    def learning(self):
        self.fsm.place_to_pose()

    def learning_run(self):
        self.fsm.place_to_pose_run()

    def state_place_to_pose(self, event):
        rospy.logdebug('Current state: state_place_to_pose')
        if not self.brain.check_robot():
            return
        self.place_object_to_pose(self.brain.instruction)

    def state_learning_place_to_pose(self, event):
        rospy.logdebug('Current state: state_learning_place_to_pose')

    def state_learning_place_to_pose_run(self, event):
        rospy.logdebug('Current state: state_learning_place_to_pose_run')
        instruction = self.brain.state_manager.state.program_current_item  # type: ProgramItem
        self.place_object_to_pose(
            instruction,
            update_state_manager=False,
            get_ready_after_place=True)

    def place_object_to_pose(
            self, instruction, update_state_manager=True, get_ready_after_place=False):

        if not self.brain.ph.is_pose_set(self.brain.block_id, instruction.id):
            if update_state_manager:
                self.brain.state_manager.update_program_item(
                    self.brain.ph.get_program_id(), self.brain.block_id, instruction)
            self.fsm.error(severity=ArtBrainErrorSeverities.ERROR,
                           error=ArtBrainErrors.ERROR_PLACE_POSE_NOT_DEFINED)

            return
        else:
            if len(instruction.ref_id) < 1:
                if update_state_manager:
                    self.brain.state_manager.update_program_item(
                        self.brain.ph.get_program_id(), self.brain.block_id, instruction)
                self.fsm.error(
                    severity=ArtBrainErrorSeverities.ERROR,
                    error=ArtBrainErrors.ERROR_NO_PICK_INSTRUCTION_ID_FOR_PLACE)

                return
            obj_type = self.brain.ph.get_object(self.brain.block_id, instruction.id)[0][0]

            arm_id = self.brain.robot.select_arm_for_place(
                obj_type, instruction.ref_id)
            if arm_id is None:
                if update_state_manager:
                    self.brain.state_manager.update_program_item(
                        self.brain.ph.get_program_id(), self.brain.block_id, instruction)
                self.fsm.error(severity=ArtBrainErrorSeverities.WARNING,
                               error=InterfaceState.ERROR_GRIPPER_NOT_HOLDING_SELECTED_OBJECT)
                return
            if update_state_manager:
                self.brain.state_manager.update_program_item(
                    self.brain.ph.get_program_id(), self.brain.block_id, instruction, {
                        "SELECTED_OBJECT_ID": self.brain.robot.get_arm_holding_object(arm_id).object_id})
            place_pose = self.brain.ph.get_pose(self.brain.block_id, instruction.id)[0][0]

            severity, error, _ = self.brain.robot.place_object_to_pose(
                place_pose, arm_id)
            if error is not None:
                if error is not ArtBrainErrors.ERROR_ROBOT_HALTED:
                    self.brain.try_robot_arms_get_ready([arm_id])
                else:
                    self.fsm.error(severity=severity, error=error, halted=True)
                    return
                self.fsm.error(severity=severity, error=error)
            else:
                if get_ready_after_place:
                    self.brain.try_robot_arms_get_ready([arm_id])
                self.fsm.done(success=True)
