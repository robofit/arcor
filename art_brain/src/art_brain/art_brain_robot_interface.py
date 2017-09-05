from art_gripper import ArtGripper
from brain_utils import ArtBrainErrors, ArtBrainErrorSeverities
import abc
import rospy
from art_msgs.msg import InterfaceState, PickPlaceGoal
import math
from std_srvs.srv import TriggerRequest, TriggerResponse


class ArtBrainRobotInterface:

    __metaclass__ = abc.ABCMeta

    def __init__(self):
        self._arms = []
        self.halted = False

    def pick_object(self, obj, pick_instruction_id, arm_id=None, pick_only_y_axis=False, feeder=False):
        print "pick_object"
        print arm_id
        if arm_id is None:
            return ArtBrainErrorSeverities.ERROR, ArtBrainErrors.ERROR_GRIPPER_NOT_DEFINED, None
        arm = self.get_arm_by_id(arm_id)
        print arm
        severity, error = self.check_arm_for_pick(arm)
        if error is not None:
            return severity, error, arm_id

        goal = PickPlaceGoal()
        goal.object = obj.object_id
        if feeder:
            goal.operation = goal.PICK_FROM_FEEDER
        else:
            goal.operation = goal.PICK_OBJECT_ID
        goal.keep_orientation = False
        goal.pick_only_y_axis = pick_only_y_axis
        rospy.logdebug("Picking object with ID: " + str(obj.object_id))
        arm.pp_client.send_goal(goal)
        arm.pp_client.wait_for_result()
        # TODO: make some error msg etc
        rospy.logdebug('Results from p&p server')
        rospy.logdebug("result: " + str(arm.pp_client.get_result()))
        rospy.logdebug("status: " + arm.pp_client.get_goal_status_text())
        rospy.logdebug("state: " + str(arm.pp_client.get_state()))

        if arm.pp_client.get_result().result == 0:
            arm.holding_object = obj
            arm.last_pick_instruction_id = pick_instruction_id
            return None, None, arm_id
        else:
            return ArtBrainErrorSeverities.WARNING, ArtBrainErrors.ERROR_PICK_FAILED, arm_id

    def place_object_to_pose(self, place_pose, arm_id, objects_frame_id="/marker", pick_only_y_axis=False):
        if arm_id is None:
            return ArtBrainErrorSeverities.ERROR, ArtBrainErrors.ERROR_GRIPPER_NOT_DEFINED, None
        arm = self.get_arm_by_id(arm_id)  # type: ArtGripper
        goal = PickPlaceGoal()
        goal.operation = goal.PLACE_TO_POSE
        goal.object = arm.holding_object.object_id
        # TODO (Kapi) check if it is safe to place the object
        #  if not self.check_place_pose(place_pose, obj):
        #    return False
        # TODO how to decide between 180 and 90 deg?
        # allow object to be rotated by 90 deg around z axis
        if not pick_only_y_axis:
            goal.z_axis_angle_increment = 3.14 / 2
        else:
            goal.z_axis_angle_increment = 3.14

        goal.pose = place_pose
        goal.pose.header.stamp = rospy.Time.now()
        goal.pose.header.frame_id = objects_frame_id
        # TODO: how to deal with this?
        goal.pose.pose.position.z = 0.09  # + obj.bbox.dimensions[2]/2

        if pick_only_y_axis:
            goal.pose.pose.orientation.x = math.sqrt(0.5)
            goal.pose.pose.orientation.w = math.sqrt(0.5)
        rospy.logdebug("Place pose: " + str(goal.pose))
        arm.pp_client.send_goal(goal)
        arm.pp_client.wait_for_result()
        rospy.logdebug("Placing object with ID: " + str(arm.holding_object.object_id))
        if arm.pp_client.get_result().result == 0:
            arm.holding_object = None
            return None, None, arm_id
        else:
            return ArtBrainErrorSeverities.WARNING, ArtBrainErrors.ERROR_PLACE_FAILED, None

    def move_arm_to_pose(self, pose, arm_id=None):
        if arm_id is None:
            return ArtBrainErrorSeverities.ERROR, ArtBrainErrors.ERROR_GRIPPER_NOT_DEFINED, None
        arm = self.get_arm_by_id(arm_id)

        if not arm.move_to_pose(pose):
            return ArtBrainErrorSeverities.WARNING, ArtBrainErrors.ERROR_GRIPPER_MOVE_FAILED, arm_id
        else:
            return None, None, arm_id

    def arms_get_ready(self, arm_ids=None):
        if arm_ids is None:
            for arm in self._arms:
                arm.get_ready()
        else:
            for arm_id in arm_ids:
                arm = self.get_arm_by_id(arm_id)
                arm.get_ready()
        return

    def arm_prepare_for_interaction(self, arm_id=None):
        if arm_id is None:
            for arm in self._arms:  # type: ArtGripper
                severity, error, arm_id = self.arm_prepare_for_interaction(arm.arm_id)
                if error is not None:
                    return severity, error, arm_id
            else:
                return None, None, None

        else:
            arm = self.get_arm_by_id(arm_id)  # type: ArtGripper
            severity, error = arm.move_to_user()
            if error is not None:
                return severity, error, arm_id
            severity, error = arm.interaction_on()
            if error is not None:
                return severity, error, arm_id
            return None, None, arm_id

    def arm_get_ready_after_interaction(self, arm_id=None):
        if arm_id is None:
            for arm in self._arms:  # type: ArtGripper
                severity, error, arm_id = self.arm_get_ready_after_interaction(arm.arm_id)
                if error is not None:
                    return severity, error, arm_id
            else:
                return None, None, None

        else:
            arm = self.get_arm_by_id(arm_id)  # type: ArtGripper
            severity, error = arm.interaction_off()
            if error is not None:
                return severity, error, arm_id
            severity, error = arm.get_ready()
            if error is not None:
                return severity, error, arm_id
            return None, None, arm_id

    def check_arm_for_pick(self, arm):
        severity, error = self.check_arm(arm)
        if error is not None:
            return severity, error
        if arm.holding_object is not None:
            rospy.logwarn(
                "Pick: gripper " +
                arm.name +
                " already holding an object (" +
                arm.holding_object.object_id +
                ")")
            return ArtBrainErrorSeverities.WARNING, ArtBrainErrors.ERROR_OBJECT_IN_GRIPPER

        return None, None

    def check_arm_for_place(self, arm):
        severity, error = self.check_arm(arm)
        if error is not None:
            return severity, error
        if arm.holding_object is None:
            return ArtBrainErrorSeverities.WARNING, ArtBrainErrors.ERROR_GRIPPER_NOT_HOLDING_SELECTED_OBJECT
        return None, None

    def check_arm(self, arm):
        if arm is None:
            rospy.logerr("No gripper!")
            return ArtBrainErrorSeverities.SEVERE, ArtBrainErrors.ERROR_NO_GRIPPER_AVAILABLE

        if arm.pp_client is None:
            rospy.logerr("No pick place client!")
            return ArtBrainErrorSeverities.SEVERE, ArtBrainErrors.ERROR_GRIPPER_PP_CLIENT_MISSING

        if not arm.pp_client.wait_for_server(rospy.Duration(2)):
            rospy.logerr("Pick place server is not running!")
            return ArtBrainErrorSeverities.WARNING, ArtBrainErrors.ERROR_PICK_PLACE_SERVER_NOT_READY

        return None, None

    @abc.abstractmethod
    def select_arm_for_pick(self, obj_id, objects_frame_id, tf_listener):
        return

    @abc.abstractmethod
    def select_arm_for_pick_from_feeder(self, pick_pose, tf_listener):
        return

    def select_arm_for_place(self, pick_instruction_ids):
        for arm in self._arms:
            if arm.last_pick_instruction_id in pick_instruction_ids:
                return arm.arm_id
        else:
            return None

    def get_arm_by_id(self, arm_id):
        """

        :param arm_id:
        :return:
        :rtype ArtGripper
        """
        for arm in self._arms:
            if arm.arm_id == arm_id:
                return arm
        else:
            return None

    def init_arms(self, arms=None):
        if arms is None:
            for arm in self._arms:
                arm.re_init()
        else:
            for arm in arms:
                arm.re_init()

    def set_halted(self, halted):
        self.halted = halted

    def is_halted(self):
        return self.halted
