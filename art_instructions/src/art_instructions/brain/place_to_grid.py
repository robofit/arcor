from art_instructions.brain import BrainFSM, BrainInstruction
from transitions import State
import rospy
from art_brain import ArtBrainUtils
from art_brain import ArtBrainErrors, ArtBrainErrorSeverities


class PlaceToGrid(BrainInstruction):
    pass


class PlaceToGridLearn(PlaceToGrid):
    pass


class PlaceToGridRun(PlaceToGrid):
    pass


class PlaceToGridFSM(BrainFSM):
    states = [
        State(name='place_to_grid', on_enter=[
            'state_update_program_item', 'check_robot_in', 'state_place_to_grid'],
            on_exit=['check_robot_out']),
        State(name='learning_place_to_grid', on_enter=[
            'learning_load_block_id', 'state_learning_place_to_grid'], on_exit=[]),
        State(name='learning_place_to_grid_run', on_enter=[
            'check_robot_in', 'learning_load_block_id', 'state_learning_place_to_grid_run'],
            on_exit=['check_robot_out']),
    ]

    transitions = [
        ('place_to_grid', 'program_run', 'place_to_grid'),
        ('done', 'place_to_grid', 'program_load_instruction'),
        ('error', 'place_to_grid', 'program_error'),
        ('place_to_grid', 'learning_run', 'learning_place_to_grid'),
        ('done', 'learning_place_to_grid', 'learning_step_done'),
        ('error', 'learning_place_to_grid', 'learning_step_error'),
        ('place_to_grid_run', 'learning_run', 'learning_place_to_grid_run'),
        ('done', 'learning_place_to_grid_run', 'learning_run'),
        ('error', 'learning_place_to_grid_run', 'learning_step_error')
    ]

    state_functions = [
        'state_place_to_grid',
        'state_learning_place_to_grid',
        'state_learning_place_to_grid_run'
    ]

    def run(self):
        self.fsm.place_to_grid()

    def learning(self):
        self.fsm.place_to_grid()

    def learning_run(self):
        self.fsm.place_to_grid_run()

    def state_place_to_grid(self, event):
        rospy.logdebug('Current state: state_place_to_grid')
        if not self.brain.check_robot():
            return
        self.place_object_to_grid(self.brain.instruction)

    def state_learning_place_to_grid(self, event):
        rospy.logdebug('Current state: state_learning_place_to_grid')

    def state_learning_place_to_grid_run(self, event):
        rospy.logdebug('Current state: state_learning_place_to_grid_run')

    def place_object_to_grid(
            self, instruction, update_state_manager=True, get_ready_after_place=True):
        rospy.logerr(
            "DO NOT USE, DEPRECATED! (place_object_to_grid in node.py)")
        pose = ArtBrainUtils.get_place_pose(instruction)

        if pose is None or len(pose) < 1:
            self.fsm.error(severity=ArtBrainErrorSeverities.ERROR,
                           error=ArtBrainErrors.ERROR_NOT_ENOUGH_PLACE_POSES)
            if update_state_manager:
                self.brain.state_manager.update_program_item(
                    self.brain.ph.get_program_id(), self.brain.block_id, instruction)
            return
        else:
            if len(instruction.ref_id) < 1:
                self.fsm.error(
                    severity=ArtBrainErrorSeverities.ERROR,
                    error=ArtBrainErrors.ERROR_NO_PICK_INSTRUCTION_ID_FOR_PLACE)
                if update_state_manager:
                    self.brain.state_manager.update_program_item(
                        self.brain.ph.get_program_id(), self.brain.block_id, instruction)
                return
            rospy.logdebug(self.brain.instruction)
            gripper = self.brain.get_gripper_by_pick_instruction_id(
                instruction.ref_id)

            if not self.brain.check_gripper_for_place(gripper):
                return

            if gripper.holding_object is None:
                rospy.logerr("Robot is not holding selected object")
                self.fsm.error(
                    severity=ArtBrainErrorSeverities.WARNING,
                    error=ArtBrainErrors.ERROR_GRIPPER_NOT_HOLDING_SELECTED_OBJECT)
                if update_state_manager:
                    self.brain.state_manager.update_program_item(
                        self.brain.ph.get_program_id(), self.brain.block_id, instruction)
                return
            if update_state_manager:
                self.brain.state_manager.update_program_item(
                    self.brain.ph.get_program_id(), self.brain.block_id, instruction,
                    {"SELECTED_OBJECT_ID": gripper.holding_object.object_id})

            if self.brain.place_object(gripper.holding_object,
                                       pose[0], gripper, pick_only_y_axis=False):
                instruction.pose.pop(0)
                gripper.holding_object = None
                if get_ready_after_place:
                    gripper.get_ready()
                if len(instruction.pose) > 0:
                    self.fsm.done(success=True)
                else:
                    self.fsm.error(severity=ArtBrainErrorSeverities.ERROR,
                                   error=ArtBrainErrors.ERROR_NOT_ENOUGH_PLACE_POSES)
            else:
                gripper.get_ready()
                self.fsm.error(severity=ArtBrainErrorSeverities.WARNING,
                               error=ArtBrainErrors.ERROR_PLACE_FAILED)
