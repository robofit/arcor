from art_brain_robot_interface import ArtBrainRobotInterface
from art_gripper import ArtGripper


class ArtDobotInterface(ArtBrainRobotInterface):

    DOBOT_ARM = 0

    def __init__(self, robot_helper):
        super(ArtDobotInterface, self).__init__(robot_parameters, robot_ns)

        self._arms.append(ArtGripper(self.DOBOT_ARM, "Dobot arm", True, False, "/art/dobot/pp_client", None,
                                     None, None,
                                     "/art/dobot/get_ready", None))

    def select_arm_for_pick(self, obj_id, objects_frame_id, tf_listener):
        return self.DOBOT_ARM

    def select_arm_for_pick_from_feeder(self, pick_pose, tf_listener):
        return self.DOBOT_ARM
