from art_brain_robot_interface import ArtBrainRobotInterface
from geometry_msgs.msg import PoseStamped
import rospy
from art_gripper import ArtGripper


class ArtPr2Interface(ArtBrainRobotInterface):

    BOTH_ARM = 2
    LEFT_ARM = 0
    RIGHT_ARM = 1

    def __init__(self, gripper_usage=BOTH_ARM):

        super(ArtPr2Interface, self).__init__()
        self.gripper_usage = gripper_usage
        self._arms = [
            ArtGripper(self.LEFT_ARM, "PR2 Left", True, True, "/art/pr2/left_arm/pp", "/art/pr2/left_arm/manipulation",
                       "/art/pr2/left_arm/interaction/on", "/art/pr2/left_arm/interaction/off",
                       "/art/pr2/left_arm/get_ready", "/art/pr2/left_arm/move_to_user"),
            ArtGripper(self.RIGHT_ARM, "PR2 Right", True, True, "/art/pr2/right_arm/pp", "/art/pr2/right_arm/manipulation",
                       "/art/pr2/right_arm/interaction/on", "/art/pr2/right_arm/interaction/off",
                       "/art/pr2/right_arm/get_ready", "/art/pr2/right_arm/move_to_user")
        ]

    def select_arm_for_pick(self, obj, objects_frame_id, tf_listener):
        left_arm = self._arms[self.LEFT_ARM]  # type: ArtGripper
        right_arm = self._arms[self.RIGHT_ARM]  # type: ArtGripper
        if left_arm.holding_object and right_arm.holding_object:
            return None
        if self.gripper_usage == self.LEFT_ARM or right_arm.holding_object:
            return self.LEFT_ARM
        elif self.gripper_usage == self.RIGHT_ARM or left_arm.holding_object:
            return self.RIGHT_ARM

        if tf_listener.frameExists(
                "/base_link") and tf_listener.frameExists(objects_frame_id):
            if obj is not None:
                obj_pose = PoseStamped()
                obj_pose.pose = obj.pose
                obj_pose.header = objects_frame_id
                # exact time does not matter in this case
                obj_pose.header.stamp = rospy.Time(0)
                tf_listener.waitForTransform(
                    '/base_link',
                    obj_pose.header.frame_id,
                    obj_pose.header.stamp,
                    rospy.Duration(1))
                obj_pose = tf_listener.transformPose(
                    '/base_link', obj_pose)
                if obj_pose.pose.position.y < 0:
                    return self.RIGHT_ARM
                else:
                    return self.LEFT_ARM
        return self.LEFT_ARM
