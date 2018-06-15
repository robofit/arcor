from art_instructions.brain import BrainFSM, BrainInstruction
from transitions import State
import rospy
from std_srvs.srv import Trigger, TriggerRequest
from std_msgs.msg import Bool
from art_brain import ArtBrainErrors, ArtBrainErrorSeverities, ArtBrainUtils


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
            'check_robot_in', 'state_visual_inspection'],
            on_exit=['check_robot_out']),
        State(name='learning_visual_inspection', on_enter=[
            'check_robot_in', 'learning_load_block_id', 'state_learning_visual_inspection'],
            on_exit=['check_robot_out', 'state_learning_visual_inspection_exit']),
        State(name='learning_visual_inspection_run', on_enter=[
            'check_robot_in', 'learning_load_block_id', 'state_learning_visual_inspection_run'],
            on_exit=['check_robot_out'])
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
        'state_learning_visual_inspection',
        'state_learning_visual_inspection_exit'
    ]

    def __init__(self, *args, **kwargs):

        super(VisualInspectionFSM, self).__init__(*args, **kwargs)

        self.visual_inspection_srv = ArtBrainUtils.create_service_client('/art/visual_inspection/start', Trigger)
        self.visual_inspection_result_sub = rospy.Subscriber(
            '/art/visual_inspection/result', Bool, self.visual_inspection_result_cb, queue_size=1)
        self.visual_inspection_result = None

    def run(self):
        self.fsm.visual_inspection()

    def learning(self):
        self.fsm.visual_inspection()

    def learning_run(self):
        self.fsm.visual_inspection_run()

    def state_visual_inspection(self, event):
        rospy.logdebug('Current state: state_visual_inspection')
        self.brain.state_manager.update_program_item(
            self.brain.ph.get_program_id(), self.brain.block_id, self.brain.instruction)
        self.visual_inspection()

    def state_learning_visual_inspection_run(self, event):
        rospy.logdebug('Current state: state_learning_visual_inspection_run')
        self.visual_inspection(get_ready_after=True)

    def state_learning_visual_inspection(self, event):
        rospy.logdebug('Current state: state_learning_visual_inspection')
        severity, error, arm_id = self.brain.robot.arm_prepare_for_interaction()
        if error is not None:
            rospy.logerr(
                "Failed to prepare gripper " +
                str(arm_id) +
                " for interaction: " +
                str(error))
            self.brain.robot.arm_get_ready_after_interaction()
            self.fsm.error(severity=severity,
                           error=error)

    def state_learning_visual_inspection_exit(self, event):
        rospy.logdebug('Current state: state_learning_visual_inspection_exit')
        severity, error, arm_id = self.brain.robot.arm_get_ready_after_interaction()
        if error is not None:
            rospy.logerr(
                "Failed to get ready gripper " +
                str(arm_id) +
                " after interaction: " +
                str(error))
            self.fsm.error(severity=severity,
                           error=error)

    def visual_inspection(self, get_ready_after=False):
        if not self.brain.check_robot():
            return
        arm_id = self.brain.robot.select_arm_for_visual_inspection()
        camera_pose, _ = self.brain.ph.get_pose(self.brain.block_id, self.brain.instruction.id)
        if camera_pose is None:
            self.fsm.error(severity=ArtBrainErrorSeverities.ERROR,
                           error=ArtBrainErrors.ERROR_PICK_POSE_NOT_SELECTED)
        else:
            camera_pose = camera_pose[0]
        severity, error, arm_id = self.brain.robot.move_arm_to_pose(
            camera_pose, arm_id, picking=False)
        if error is not None:
            if error is not ArtBrainErrors.ERROR_ROBOT_HALTED:
                self.brain.try_robot_arms_get_ready([arm_id])
            else:
                self.fsm.error(severity=severity, error=error, halted=True)
                return
            self.fsm.error(severity=severity, error=error)
            return

        resp = self.visual_inspection_srv.call(TriggerRequest())

        if not resp.success:
            pass
            # TODO: solve error
        while self.visual_inspection_result is None and not rospy.is_shutdown():
            rospy.sleep(0.2)
        if rospy.is_shutdown():
            return
        if get_ready_after:
            self.brain.try_robot_arms_get_ready([arm_id])
        if self.visual_inspection_result:
            self.visual_inspection_result = None
            self.fsm.done(success=True)
        else:
            self.visual_inspection_result = None
            self.fsm.done(success=False)

    def visual_inspection_result_cb(self, success):
        self.visual_inspection_result = success.data
