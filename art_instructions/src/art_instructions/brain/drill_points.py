from art_instructions.brain import BrainFSM, BrainInstruction
from transitions import State
from art_brain import ArtBrainErrors, ArtBrainErrorSeverities, ArtBrainUtils
import rospy
import copy
from art_msgs.srv import ObjectFlagSetRequest


class DrillPointsFSM(BrainFSM):
    states = [
        State(name='drill_points', on_enter=[
            'state_update_program_item', 'check_robot_in', 'state_drill_points'], on_exit=['check_robot_out']),
        State(name='learning_drill_points', on_enter=[
            'check_robot_in', 'learning_load_block_id', 'state_learning_drill_points'], on_exit=[
            'check_robot_out', 'state_learning_drill_points_exit']),
        State(name='learning_drill_points_run', on_enter=[
            'check_robot_in', 'learning_load_block_id', 'state_learning_drill_points_run'],
            on_exit=['check_robot_out']),

    ]

    transitions = [
        ('drill_points', 'program_run', 'drill_points'),
        ('done', 'drill_points', 'program_load_instruction'),
        ('error', 'drill_points', 'program_error'),
        ('drill_points', 'learning_run', 'learning_drill_points'),
        ('done', 'learning_drill_points', 'learning_step_done'),
        ('error', 'learning_drill_points', 'learning_step_error'),
        ('drill_points_run', 'learning_run', 'learning_drill_points_run'),
        ('done', 'learning_drill_points_run', 'learning_run'),
        ('error', 'learning_drill_points_run', 'learning_step_error')

    ]

    state_functions = [
        'state_drill_points',
        'state_learning_drill_points',
        'state_learning_drill_points_run',
        'state_learning_drill_points_exit'
    ]

    def __init__(self, *args, **kwargs):

        super(DrillPointsFSM, self).__init__(*args, **kwargs)

    def run(self):
        self.fsm.drill_points()

    def learning(self):
        self.fsm.drill_points()

    def learning_run(self):
        self.fsm.drill_points_run()

    def state_drill_points(self, event):
        rospy.logdebug('Current state: state_drill_points')
        self.drill_points(self.brain.instruction)

    def state_learning_drill_points_exit(self, event):
        rospy.logdebug('Current state: state_learning_drill_points_exit')
        severity, error, arm_id = self.brain.robot.arm_get_ready_after_interaction()
        if error is not None:
            rospy.logerr(
                "Failed to get ready gripper " +
                str(arm_id) +
                " after interaction: " +
                str(error))
            self.fsm.error(severity=severity,
                           error=error)

    def state_learning_drill_points(self, event):
        rospy.logdebug('Current state: state_learning_drill_points')

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

    def state_learning_drill_points_run(self, event):
        rospy.logdebug('Current state: state_learning_drill_points_run')
        instruction = self.brain.state_manager.state.program_current_item  # type: ProgramItem

        self.drill_points(instruction, set_drilled_flag=False)
        self.brain.try_robot_arms_get_ready()

    def drill_points(self, instruction, set_drilled_flag=True):

        # TODO drill_enabled() je metoda ArtRobotArmHelper - jenze tady jeste nevim ktere rameno se bude pouzivat
        # TODO ERROR_NOT_IMPLEMENTED -> myslim ze by bylo lepsi zkontrolovat program pri pozadavku na spusteni -
        # jestli neobsahuje robotem nepodporovane instrukce a pak uz se tim nezabyvat - ke spusteni programu s
        # instrukci co robot nepodporuje by vubec nemelo dojit
        # if not self.rh.drill_enabled():
        #    self.fsm.error(severity=ArtBrainErrorSeverities.ERROR,
        #                   error=ArtBrainErrors.ERROR_NOT_IMPLEMENTED)
        #    return

        if not self.brain.check_robot():
            return
        objects, _ = self.brain.ph.get_object(self.brain.block_id, instruction.id)

        # TODO tohle nemuze nastat - kontroluje program helper
        if len(objects) < 1:
            self.fsm.error(severity=ArtBrainErrorSeverities.ERROR,
                           error=ArtBrainErrors.ERROR_OBJECT_NOT_DEFINED)
            return
        obj_type = self.brain.ph.get_object(self.brain.block_id, instruction.id)[0][0]

        obj_to_drill = None
        objects_in_polygon = ArtBrainUtils.get_objects_in_polygon(
            obj_type, self.brain.ph.get_polygon(
                self.brain.block_id, instruction.id)[0][0], self.brain.objects)
        if not objects_in_polygon:
            self.fsm.error(severity=ArtBrainErrorSeverities.WARNING,
                           error=ArtBrainErrors.ERROR_OBJECT_MISSING_IN_POLYGON)
            return

        for obj in objects_in_polygon:

            drilled = False
            for flag in obj.flags:
                if flag.key == "drilled" and flag.value == "true":
                    rospy.logdebug(
                        "Object " + obj.object_id + " already drilled.")
                    drilled = True
                    break
            if drilled:
                continue

            obj_to_drill = obj
            break

        if obj_to_drill is None:
            rospy.loginfo("All objects in polygon seems to be drilled.")
            # self.try_robot_arms_get_ready([arm_id])
            self.fsm.done(success=False)
            return

        arm_id = self.brain.robot.select_arm_for_drill(
            obj_to_drill, self.brain.objects.header.frame_id, self.brain.tf_listener)
        if arm_id != self.brain.last_drill_arm_id:
            if self.brain.last_drill_arm_id is not None:
                self.brain.robot.arms_get_ready([self.brain.last_drill_arm_id])
            self.brain.last_drill_arm_id = copy.deepcopy(arm_id)

        rospy.loginfo("Drilling object: " + obj_to_drill.object_id)

        poses = self.brain.ph.get_pose(self.brain.block_id, instruction.id)[0]

        for hole_number, pose in enumerate(poses):
            rospy.loginfo("Hole number: " + str(hole_number + 1) +
                          " (out of: " + str(len(poses)) + ")")

            if self.brain.program_pause_request or self.brain.program_paused:
                self.brain.program_pause_request = False
                self.brain.program_paused = True
                r = rospy.Rate(2)
                while self.brain.program_paused:
                    r.sleep()
            self.brain.state_manager.update_program_item(
                self.brain.ph.get_program_id(), self.brain.block_id, instruction, {
                    "SELECTED_OBJECT_ID": obj_to_drill.object_id, "DRILLED_HOLE_NUMBER": str(hole_number + 1)})

            severity, error, arm_id = self.brain.robot.drill_point(
                arm_id, [pose], obj_to_drill, "TODO", drill_duration=0)
            if error:
                rospy.logwarn("Drilling failed...")
                self.fsm.error(severity=severity,
                               error=error)
                return

        if set_drilled_flag:
            req = ObjectFlagSetRequest()
            req.object_id = obj_to_drill.object_id
            req.flag.key = "drilled"
            req.flag.value = "true"

            ret = self.brain.set_object_flag_srv_client.call(req)

            if not ret.success:
                rospy.logerr("Failed to set flag!")

            st = copy.deepcopy(self.brain.objects.header.stamp)
            # need to wait (for new message from tracker) until flag is really set
            # otherwise object might be drilled again...
            # TODO check object flags insted of stamp? or remember that object
            # was drilled e.g. in self.drilled_objects ?
            while self.brain.objects.header.stamp == st:
                rospy.sleep(0.1)

        rospy.loginfo("Object drilled: " + obj_to_drill.object_id)

        self.fsm.done(success=True)
