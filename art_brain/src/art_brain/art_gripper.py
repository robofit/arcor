from brain_utils import ArtBrainUtils, ArtBrainErrors, ArtBrainErrorSeverities
import actionlib
from art_msgs.msg import PickPlaceAction, ArmNavigationAction, ArmNavigationGoal, ArmNavigationResult
from std_srvs.srv import Empty, Trigger
import rospy


class ArtGripper(object):

    def __init__(self, arm_id, name, pick_place_enabled, drill_enabled, pp_client=None, manipulation_client=None, interaction_on_client=None,
                 interaction_off_client=None, get_ready_client=None, move_to_user_client=None):
        self.arm_id = arm_id
        self.name = name

        self.pp_client_name = pp_client
        self.manip_client_name = manipulation_client
        if pick_place_enabled:
            self.pp_client = actionlib.SimpleActionClient(
                self.pp_client_name, PickPlaceAction)
            rospy.loginfo("Waiting for " + str(name) + "'s gripper pick&place action client")
            self.pp_client.wait_for_server()
	    rospy.loginfo("Connected to " + str(name) + "'s gripper pick&place action client")
        else:
            self.pp_client = None
        if drill_enabled:
            self.manip_client = actionlib.SimpleActionClient(
                self.manip_client_name, ArmNavigationAction)
            rospy.loginfo("Waiting for " + str(name) + "'s gripper manipulation action client")
            self.manip_client.wait_for_server()
            rospy.loginfo("Connected to " + str(name) + "'s gripper pick&place action client") 
        else:
            self.manip_client = None

        self.holding_object = None
        self.last_pick_instruction_id = None
        self.group_name = name
        if interaction_on_client is not None:
            self.interaction_on_client = ArtBrainUtils.create_service_client(
                interaction_on_client, Empty)
        else:
            self.interaction_on_client = None
        if interaction_off_client is not None:
            self.interaction_off_client = ArtBrainUtils.create_service_client(
                interaction_off_client, Empty)
        else:
            self.interaction_off_client = None
        if get_ready_client is not None:
            self.get_ready_client = ArtBrainUtils.create_service_client(
                get_ready_client, Trigger)
        else:
            self.get_ready_client = None
        if move_to_user_client is not None:
            self.move_to_user_client = ArtBrainUtils.create_service_client(
                move_to_user_client, Trigger)
        else:
            self.move_to_user_client = None

        rospy.loginfo("Arm " + str(name) + " ready.")

    def re_init(self):
        self.last_pick_instruction_id = None
        self.holding_object = None

    def get_ready(self):
        self.get_ready_client.call()

    def move_to_user(self):
        self.move_to_user_client.call()

    def interaction_on(self):
        self.interaction_on_client.call()

    def interaction_off(self):
        self.interaction_off()

    def move_through_poses(self, poses):
        if self.manip_client is None:
            return False
        goal = ArmNavigationGoal()
        goal.operation = goal.MOVE_THROUGH_POSES
        goal.poses = poses
        self.manip_client.send_goal(goal)
        rospy.sleep(1)
        self.manip_client.wait_for_result()
        if self.manip_client.get_result().result == ArmNavigationResult.SUCCESS:
            return True
        else:
            return False

    def touch_poses(self, object_id, poses, drill_duration=0):
        if self.manip_client is None:
            return False
        goal = ArmNavigationGoal()
        goal.object = object_id
        goal.operation = goal.TOUCH_POSES
        goal.drill_duration = drill_duration
        goal.poses = poses
        for pose in goal.poses:  # type: PoseStamped
            pose.header.frame_id = object_id
        self.manip_client.send_goal(goal)
        rospy.sleep(1)
        self.manip_client.wait_for_result()
        if self.manip_client.get_result().result == ArmNavigationResult.SUCCESS:
            return True
        else:
            return False

    def move_to_pose(self, pose):
        if self.manip_client is None:
            return False
        goal = ArmNavigationGoal()
        goal.poses = [pose]
        goal.operation = ArmNavigationGoal.MOVE_THROUGH_POSES
        self.manip_client.send_goal(goal)
        rospy.sleep(1)
        self.manip_client.wait_for_result()
        if self.manip_client.get_result().result == ArmNavigationResult.SUCCESS:
            return True
        else:
            return False

    def prepare_for_interaction(self):
        if self.move_to_user_client is None or self.interaction_on_client is None:
            return False, ArtBrainErrors.ERROR_LEARNING_GRIPPER_INTERACTION_MODE_SWITCH_FAILED
        result = self.move_to_user_client.call()

        if not result.success:
            rospy.logwarn("Can't move gripper to the user: " +
                          str(result.message))
            return False, ArtBrainErrors.ERROR_GRIPPER_MOVE_FAILED
        result = self.interaction_on_client.call()
        if not result:
            rospy.logwarn(
                "Can't change gripper interaction state: " + str(result.message))
            # TODO: check arm state, inform user
            return False, ArtBrainErrors.ERROR_LEARNING_GRIPPER_INTERACTION_MODE_SWITCH_FAILED
        return True, None
