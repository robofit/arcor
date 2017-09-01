from art_brain_robot_interface import ArtBrainRobotInterface
from art_gripper import ArtGripper


class ArtDobotInterface(ArtBrainRobotInterface):

    DOBOT_ARM = 0

    def __init__(self):
        super(ArtDobotInterface, self).__init__()
        ArtGripper(self.DOBOT_ARM, "Dobot arm", True, False, "/art/pr2/left/pp", "/art/pr2/left/manipulation",
                   "/art/pr2/left/interaction/on", "/art/pr2/left/interaction/off",
                   "/art/pr2/left/get_ready", "/art/pr2/left/move_to_user"),

    def select_arm_for_pick(self, obj_id, objects_frame_id, tf_listener):
        return self.DOBOT_ARM

    def select_arm_for_pick_from_feeder(self, pick_pose, tf_listener):
        return self.DOBOT_ARM
