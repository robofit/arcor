from art_instructions.brain import BrainFSM, BrainInstruction
from transitions import State
import rospy
from art_brain import ArtBrainErrors, ArtBrainErrorSeverities, ArtBrainUtils
from art_msgs.msg import ProgramItem, InterfaceState
from geometry_msgs.msg import PoseStamped


class PlaceToContainerFSM(BrainFSM):
    states = [
        State(name='place_to_container', on_enter=[
            'state_update_program_item', 'check_robot_in', 'state_place_to_container'],
            on_exit=['check_robot_out']),
        State(name='learning_place_to_container', on_enter=[
            'learning_load_block_id', 'state_learning_place_to_container'], on_exit=[]),
        State(name='learning_place_to_container_run', on_enter=[
            'check_robot_in', 'learning_load_block_id', 'state_learning_place_to_container_run'],
            on_exit=['check_robot_out']),
    ]

    transitions = [
        ('place_to_container', 'program_run', 'place_to_container'),
        ('done', 'place_to_container', 'program_load_instruction'),
        ('error', 'place_to_container', 'program_error'),
        ('place_to_container', 'learning_run', 'learning_place_to_container'),
        ('done', 'learning_place_to_container', 'learning_step_done'),
        ('error', 'learning_place_to_container', 'learning_step_error'),
        ('place_to_container_run', 'learning_run', 'learning_place_to_container_run'),
        ('done', 'learning_place_to_container_run', 'learning_run'),
        ('error', 'learning_place_to_container_run', 'learning_step_error')
    ]

    state_functions = [
        'state_place_to_container',
        'state_learning_place_to_container',
        'state_learning_place_to_container_run'
    ]

    def run(self):
        self.fsm.place_to_container()

    def learning(self):
        self.fsm.place_to_container()

    def learning_run(self):
        self.fsm.place_to_container_run()

    def state_place_to_container(self, event):
        rospy.logdebug('Current state: state_place_to_container')
        if not self.brain.check_robot():
            return
        self.place_object_to_container(self.brain.instruction)

    def state_learning_place_to_container(self, event):
        rospy.logdebug('Current state: state_learning_place_to_container')

    def state_learning_place_to_container_run(self, event):
        rospy.logdebug('Current state: state_learning_place_to_container_run')
        instruction = self.brain.state_manager.state.program_current_item  # type: ProgramItem
        self.place_object_to_container(
            instruction,
            update_state_manager=False,
            get_ready_after_place=True)

    def place_object_to_container(
            self, instruction, update_state_manager=True, get_ready_after_place=False):

        container_type = self.brain.ph.get_object(self.brain.block_id, instruction.id)[0][0]

        arm_id = self.brain.robot.select_arm_for_place("", instruction.ref_id)
        if arm_id is None:
            if update_state_manager:
                self.brain.state_manager.update_program_item(
                    self.brain.ph.get_program_id(), self.brain.block_id, instruction)
            self.fsm.error(severity=ArtBrainErrorSeverities.WARNING,
                           error=InterfaceState.ERROR_GRIPPER_NOT_HOLDING_SELECTED_OBJECT)
            return

        polygon = self.brain.ph.get_polygon(self.brain.block_id, self.brain.instruction.id)[0][0]
        container = ArtBrainUtils.get_pick_obj_from_polygon(
            container_type, polygon, self.brain.objects)
        if container is None:
            self.fsm.error(severity=ArtBrainErrorSeverities.WARNING,
                           error=ArtBrainErrors.ERROR_OBJECT_MISSING_IN_POLYGON)
            return

        if update_state_manager:
            self.brain.state_manager.update_program_item(
                self.brain.ph.get_program_id(), self.brain.block_id, self.brain.instruction, {
                    "SELECTED_OBJECT_ID": self.brain.robot.get_arm_holding_object(arm_id).object_id,
                    "SELECTED_CONTAINER_ID": container.object_id})

        container_bb = self.brain.art.get_object_type(container_type).bbox.dimensions
        place_pose = PoseStamped()
        place_pose.header.frame_id = "marker"
        place_pose.pose = container.pose

        place_pose.pose.position.z += container_bb[2] / 2.0 + 0.065  # TODO plus half of bb height of the picked object
        severity, error, _ = self.brain.robot.place_object_to_pose(
            place_pose, arm_id)
        if error is not None:
            if error is not ArtBrainErrors.ERROR_ROBOT_HALTED:
                self.brain.try_robot_arms_get_ready([arm_id])
            else:
                self.fsm.error(severity=severity, error=error, halted=True)
                return
            self.fsm.error(severity=severity, error=error)
            return
        else:
            self.brain.robot.get_arm_by_id(arm_id).last_pick_instruction_id = None
            if get_ready_after_place:
                self.brain.try_robot_arms_get_ready([arm_id])
            self.fsm.done(success=True)
            return
