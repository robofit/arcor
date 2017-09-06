from art_brain_robot_interface import ArtBrainRobotInterface
from geometry_msgs.msg import PoseStamped
import rospy
from art_gripper import ArtGripper
from brain_utils import ArtBrainUtils


class ArtPr2Interface(ArtBrainRobotInterface):

    BOTH_ARM = 2
    LEFT_ARM = 0
    RIGHT_ARM = 1

    def __init__(self, robot_parameters, robot_ns, gripper_usage=BOTH_ARM):

        super(ArtPr2Interface, self).__init__(robot_parameters, robot_ns)
        self.gripper_usage = gripper_usage
        '''self._arms = [
            ArtGripper(self.LEFT_ARM, "PR2 Left", True, True, "/art/pr2/left_arm/pp", "/art/pr2/left_arm/manipulation",
                       "/art/pr2/left_arm/interaction/on", "/art/pr2/left_arm/interaction/off",
                       "/art/pr2/left_arm/get_ready", "/art/pr2/left_arm/move_to_user", gripper_link="l_gripper_tool_frame"),
            ArtGripper(self.RIGHT_ARM, "PR2 Right", True, True, "/art/pr2/right_arm/pp", "/art/pr2/right_arm/manipulation",
                       "/art/pr2/right_arm/interaction/on", "/art/pr2/right_arm/interaction/off",
                       "/art/pr2/right_arm/get_ready", "/art/pr2/right_arm/move_to_user", gripper_link="r_gripper_tool_frame")
        ]'''

    def select_arm_for_pick(self, obj, objects_frame_id, tf_listener):
        free_arm = self.select_free_arm()
        if free_arm in [None, self.LEFT_ARM, self.RIGHT_ARM]:
            return free_arm
        objects_frame_id = ArtBrainUtils.normalize_frame_id(objects_frame_id)
        if tf_listener.frameExists(
                "base_link") and tf_listener.frameExists(ArtBrainUtils.normalize_frame_id(objects_frame_id)):
            if obj is not None:
                obj_pose = PoseStamped()
                obj_pose.pose = obj.pose
                obj_pose.header = objects_frame_id
                # exact time does not matter in this case
                obj_pose.header.stamp = rospy.Time(0)
                tf_listener.waitForTransform(
                    'base_link',
                    obj_pose.header.frame_id,
                    obj_pose.header.stamp,
                    rospy.Duration(1))
                obj_pose = tf_listener.transformPose(
                    'base_link', obj_pose)
                if obj_pose.pose.position.y < 0:
                    return self.RIGHT_ARM
                else:
                    return self.LEFT_ARM
        return self.LEFT_ARM

    def select_arm_for_pick_from_feeder(self, pick_pose, tf_listener):
        pick_pose.header.frame_id = ArtBrainUtils.normalize_frame_id(pick_pose.header.frame_id)
        free_arm = self.select_free_arm()
        if free_arm in [None, self.LEFT_ARM, self.RIGHT_ARM]:
            return free_arm
        print pick_pose.header.frame_id
        print tf_listener.frameExists("base_link")
        print tf_listener.frameExists(pick_pose.header.frame_id)
        print pick_pose
        # if tf_listener.frameExists("base_link") and tf_listener.frameExists(pick_pose.header.frame_id) \
        #        and pick_pose is not None:
        pick_pose.header.stamp = rospy.Time(0)
        tf_listener.waitForTransform(
            'base_link',
            pick_pose.header.frame_id,
            pick_pose.header.stamp,
            rospy.Duration(1))
        obj_pose = tf_listener.transformPose(
            'base_link', pick_pose)
        if obj_pose.pose.position.y < 0:
            return self.RIGHT_ARM
        else:
            return self.LEFT_ARM
        return None

    def select_free_arm(self):
        left_arm = self._arms[self.LEFT_ARM] if self.gripper_usage in [self.BOTH_ARM,
                                                                       self.LEFT_ARM] else None  # type: ArtGripper
        right_arm = self._arms[self.RIGHT_ARM] if self.gripper_usage in [self.BOTH_ARM,
                                                                         self.RIGHT_ARM] else None  # type: ArtGripper
        if left_arm is None and right_arm is None:
            return None

        if self.gripper_usage == self.LEFT_ARM:
            if left_arm.holding_object:
                return None
            else:
                return self.LEFT_ARM
        elif self.gripper_usage == self.RIGHT_ARM:
            if left_arm.holding_object:
                return None
            else:
                return self.RIGHT_ARM
        elif self.gripper_usage == self.BOTH_ARM:
            if left_arm.holding_object and not right_arm.holding_object:
                return right_arm
            elif not left_arm.holding_object and right_arm.holding_object:
                return left_arm
            elif left_arm.holding_object and right_arm.holding_object:
                return None
        return self.BOTH_ARM
