from art_instructions.brain import BrainFSM, BrainInstruction
from transitions import State
from art_brain import ArtBrainErrors, ArtBrainErrorSeverities, ArtBrainUtils
import rospy
from geometry_msgs.msg import PoseStamped
import math

class PickFromFeeder(BrainInstruction):

    def __init__(self, *args, **kwargs):

        super(PickFromFeeder, self).__init__(*args, **kwargs)


class PickFromFeederLearn(PickFromFeeder):

    def __init__(self, *args, **kwargs):

        super(PickFromFeederLearn, self).__init__(*args, **kwargs)


class PickFromFeederRun(PickFromFeeder):

    def __init__(self, *args, **kwargs):

        super(PickFromFeederRun, self).__init__(*args, **kwargs)


class PickFromFeederFSM(BrainFSM):
    states = [
        State(name='pick_from_feeder', on_enter=[
            'state_update_program_item', 'check_robot_in', 'state_pick_from_feeder'],
              on_exit=['check_robot_out']),
        State(name='learning_pick_from_feeder_run', on_enter=[
            'check_robot_in', 'learning_load_block_id', 'state_learning_pick_from_feeder_run'],
              on_exit=['check_robot_out']),
        State(name='learning_pick_from_feeder', on_enter=[
            'check_robot_in', 'learning_load_block_id', 'state_learning_pick_from_feeder'],
              on_exit=['check_robot_out', 'state_learning_pick_from_feeder_exit'])
    ]

    transitions = [
        ('pick_from_feeder', 'program_run', 'pick_from_feeder'),
        ('done', 'pick_from_feeder', 'program_load_instruction'),
        ('error', 'pick_from_feeder', 'program_error'),
        ('pick_from_feeder', 'learning_run', 'learning_pick_from_feeder'),
        ('done', 'learning_pick_from_feeder', 'learning_step_done'),
        ('error', 'learning_pick_from_feeder', 'learning_step_error'),
        ('pick_from_feeder_run', 'learning_run', 'learning_pick_from_feeder_run'),
        ('done', 'learning_pick_from_feeder_run', 'learning_run'),
        ('error', 'learning_pick_from_feeder_run', 'learning_step_error')
    ]

    state_functions = [
        'state_pick_from_feeder',
        'state_learning_pick_from_feeder_run',
        'state_learning_pick_from_feeder',
        'state_learning_pick_from_feeder_exit'
    ]

    def __init__(self, *args, **kwargs):

        super(PickFromFeederFSM, self).__init__(*args, **kwargs)

    def run(self):
        self.fsm.pick_from_feeder()

    def learning(self):
        self.fsm.pick_from_feeder()

    def learning_run(self):
        self.fsm.pick_from_feeder_run()

    def state_pick_from_feeder(self, event):
        rospy.logdebug('Current state: state_pick_from_feeder')
        if not self.brain.check_robot():
            return
        self.brain.forearm_enable_srv_client.call()
        self.pick_object_from_feeder(self.brain.instruction)
        self.brain.forearm_disable_srv_client.call()

    def state_learning_pick_from_feeder(self, event):
        rospy.logdebug('Current state: state_learning_pick_from_feeder')

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

    def state_learning_pick_from_feeder_run(self, event):
        rospy.logdebug('Current state: state_learning_pick_from_feeder_run')
        instruction = self.brain.state_manager.state.program_current_item  # type: ProgramItem
        self.brain.forearm_enable_srv_client.call()
        self.pick_object_from_feeder(instruction)
        self.brain.forearm_disable_srv_client.call()

    def state_learning_pick_from_feeder_exit(self, event):
        rospy.logdebug('Current state: state_learning_pick_from_feeder_exit')
        severity, error, arm_id = self.brain.robot.arm_get_ready_after_interaction()
        if error is not None:
            rospy.logerr(
                "Failed to get ready gripper " +
                str(arm_id) +
                " after interaction: " +
                str(error))
            self.fsm.error(severity=severity,
                           error=error)

    def pick_object_from_feeder(self, instruction):

        self.brain.state_manager.update_program_item(
            self.brain.ph.get_program_id(), self.brain.block_id, instruction)

        if not self.brain.ph.is_object_set(self.brain.block_id, instruction.id):
            self.fsm.error(severity=ArtBrainErrorSeverities.ERROR,
                           error=ArtBrainErrors.ERROR_OBJECT_NOT_DEFINED)
            return
        obj_type = self.brain.ph.get_object(self.brain.block_id, instruction.id)[0][0]
        obj = ArtBrainUtils.get_pick_obj_from_feeder(obj_type)

        if not self.brain.ph.is_pose_set(self.brain.block_id, instruction.id):
            self.fsm.error(severity=ArtBrainErrorSeverities.ERROR,
                           error=ArtBrainErrors.ERROR_PICK_POSE_NOT_SELECTED)
            return
        pick_pose, _ = self.brain.ph.get_pose(self.brain.block_id, instruction.id)
        if pick_pose is None:
            self.fsm.error(severity=ArtBrainErrorSeverities.ERROR,
                           error=ArtBrainErrors.ERROR_PICK_POSE_NOT_SELECTED)
        else:
            pick_pose = pick_pose[0]
        arm_id = self.brain.robot.select_arm_for_pick_from_feeder(
            pick_pose, self.brain.tf_listener)
        severity, error, arm_id = self.brain.robot.move_arm_to_pose(
            pick_pose, arm_id, picking=True)
        if error is not None:
            if error is not ArtBrainErrors.ERROR_ROBOT_HALTED:
                self.brain.try_robot_arms_get_ready([arm_id])
            else:
                self.fsm.error(severity=severity, error=error, halted=True)
                return
            self.fsm.error(severity=severity, error=error)
            return

        start_time = rospy.Time.now()
        object_found_time = None

        pick_object = None
        pick_object_dist = None
        rospy.loginfo("Looking for: " + str(obj.object_type))

        ignored_objects = []

        while True:

            now = rospy.Time.now()

            if start_time + rospy.Duration(5.0) < now:
                rospy.logwarn("Can't find object in feeder in given time.")
                break

            if object_found_time and object_found_time + \
                    rospy.Duration(1.0) < now:
                break

            for inst in self.brain.objects.instances:  # type: ObjInstance

                if inst.object_id in ignored_objects:
                    continue

                if inst.object_type != obj.object_type:
                    continue

                # TODO read table size from some param
                # TODO on_table -> use method from some helper class (shared
                # with gui...), add it to message?
                on_table = inst.pose.position.z < 0.1 and 0 < inst.pose.position.x < 1.5

                if on_table:
                    rospy.logdebug(
                        "Ignoring 'on_table' object: " +
                        inst.object_id)
                    ignored_objects.append(inst.object_id)
                    continue

                ps = PoseStamped()
                ps.header.frame_id = self.brain.objects.header.frame_id
                ps.header.stamp = rospy.Time(0)
                ps.pose = inst.pose
                # TODO compute transform once and then only apply it
                ps = self.brain.tf_listener.transformPose(
                    self.brain.robot.get_arm_by_id(arm_id).gripper_link, ps)
                # distance in x does not matter - we want the object closest to
                # the x-axis of gripper
                dist = math.sqrt(
                    ps.pose.position.y ** 2 +
                    ps.pose.position.z ** 2)
                rospy.logdebug("Distance to object ID " +
                               inst.object_id +
                               " is: " +
                               str(dist) +
                               ", dist to gripper: " +
                               str(ps.pose.position.x))

                if dist > 0.1:
                    rospy.logdebug("Object is too far in y/z.")
                    continue

                # dist in x has to be bigger than length of the gripper
                if 0.05 < ps.pose.position.x < 0.2:

                    if pick_object_dist is None or dist < pick_object_dist:
                        object_found_time = now
                        pick_object = inst
                        pick_object_dist = dist
                        rospy.logdebug("Storing object: " + inst.object_id)

                else:
                    rospy.logdebug("Object to far in x.")

        if not pick_object:
            self.brain.try_robot_arms_get_ready([arm_id])
            self.fsm.error(severity=ArtBrainErrorSeverities.WARNING,
                           error=ArtBrainErrors.ERROR_OBJECT_MISSING)
            return

        severity, error, arm_id = self.brain.robot.pick_object(
            pick_object, instruction.id, arm_id, from_feeder=True)
        if error is not None:
            if error is not ArtBrainErrors.ERROR_ROBOT_HALTED:
                self.brain.try_robot_arms_get_ready([arm_id])
            else:
                self.fsm.error(severity=severity, error=error, halted=True)
                return
            self.fsm.error(severity=severity, error=error)
        else:
            self.fsm.done(success=True)