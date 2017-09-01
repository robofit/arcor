#!/usr/bin/env python

import rospy
from dobot.srv import \
    GetPose, GetPoseRequest, GetPoseResponse, \
    SetPTPCmd, SetPTPCmdRequest, SetPTPCmdResponse, \
    SetCmdTimeout, SetCmdTimeoutRequest, SetCmdTimeoutResponse, \
    SetQueuedCmdClear, SetQueuedCmdClearRequest, SetQueuedCmdClearResponse, \
    SetQueuedCmdStartExec, SetQueuedCmdStartExecRequest, SetQueuedCmdStartExecResponse, \
    SetEndEffectorParams, SetEndEffectorParamsRequest, SetEndEffectorParamsResponse, \
    SetPTPJointParams, SetPTPJointParamsRequest, SetPTPJointParamsResponse, \
    SetPTPCoordinateParams, SetPTPCoordinateParamsRequest, SetPTPCoordinateParamsResponse, \
    SetPTPJumpParams, SetPTPJumpParamsRequest, SetPTPJumpParamsResponse, \
    SetPTPCommonParams, SetPTPCommonParamsRequest, SetPTPCommonParamsResponse, \
    SetEndEffectorSuctionCup, SetEndEffectorSuctionCupRequest, SetEndEffectorSuctionCupResponse, \
    GetEndEffectorSuctionCup, GetEndEffectorSuctionCupRequest, GetEndEffectorSuctionCupResponse
from tf import TransformBroadcaster, TransformListener
from geometry_msgs.msg import PoseStamped, Pose
from art_msgs.msg import PickPlaceAction, PickPlaceFeedback, PickPlaceGoal, PickPlaceResult, InstancesArray, \
    ObjInstance
import actionlib
from copy import deepcopy
from art_msgs.srv import getObjectType, getObjectTypeRequest, getObjectTypeResponse
from std_srvs.srv import Trigger, TriggerRequest, TriggerResponse


class TimeoutException(Exception):
    pass


class DobotClient(object):

    def __init__(self):
        self.get_ready_srv = rospy.Service("get_ready", Trigger, self.get_ready_cb)

        self.get_object_type_srv = rospy.ServiceProxy("/art/db/object_type/get", getObjectType)

        self.get_pose_srv = rospy.ServiceProxy("/DobotServer/GetPose", GetPose)
        self.move_to_srv = rospy.ServiceProxy("/DobotServer/SetPTPCmd", SetPTPCmd)
        self.set_suction_srv = rospy.ServiceProxy("/DobotServer/SetEndEffectorSuctionCup", SetEndEffectorSuctionCup)
        self.get_suction_srv = rospy.ServiceProxy("/DobotServer/GetEndEffectorSuctionCup", GetEndEffectorSuctionCup)

        self.set_cmd_timeout_srv = rospy.ServiceProxy("/DobotServer/SetCmdTimeout", SetCmdTimeout)
        self.set_queued_cmd_clear_srv = rospy.ServiceProxy("/DobotServer/SetQueuedCmdClear", SetQueuedCmdClear)
        self.set_queued_cmd_start_exec_srv = rospy.ServiceProxy("/DobotServer/SetQueuedCmdStartExec",
                                                                SetQueuedCmdStartExec)
        self.set_ent_effector_params_srv = rospy.ServiceProxy("/DobotServer/SetEndEffectorParams", SetEndEffectorParams)
        self.set_ptp_joint_params_srv = rospy.ServiceProxy("/DobotServer/SetPTPJointParams", SetPTPJointParams)
        self.set_ptp_coord_params_srv = rospy.ServiceProxy("/DobotServer/SetPTPCoordinateParams",
                                                           SetPTPCoordinateParams)
        self.set_ptp_jump_params_srv = rospy.ServiceProxy("/DobotServer/SetPTPJumpParams", SetPTPJumpParams)
        self.set_ptp_common_params_srv = rospy.ServiceProxy("/DobotServer/SetPTPCommonParams", SetPTPCommonParams)
        self._action_name = "pp_client"
        self._as = actionlib.SimpleActionServer(self._action_name, PickPlaceAction, self.pick_place_cb, True)

        self.move_to_sub = rospy.Subscriber("move_to", PoseStamped, self.move_to_cb, queue_size=100)
        self.object_sub = rospy.Subscriber("/art/object_detector/object_filtered", InstancesArray, self.objects_cb,
                                           queue_size=1)

        self.tf_broadcaster = TransformBroadcaster()
        self.tf_listener = TransformListener()
        self._dobot_pose = Pose()

        self._objects = InstancesArray()
        self._grasped_object = ObjInstance()
        self._init_dobot()

        rospy.loginfo("Dobot ready")

    def pick_place_cb(self, goal):
        """

        Args:
            goal:

        @type goal: PickPlaceGoal

        Returns:

        """
        res = PickPlaceResult()
        if goal.operation == PickPlaceGoal.RESET:
            self._objects = None
            self._grasped_object = None
            res.result = res.SUCCESS
            self._as.set_succeeded(res)
        elif goal.operation == PickPlaceGoal.PICK_OBJECT_ID:
            res = self.pick_object_id(goal.object)
            if res.result == res.SUCCESS:
                self._as.set_succeeded(res)
            else:
                self._as.set_aborted(res)
        elif goal.operation == PickPlaceGoal.PLACE_TO_POSE:
            self.place_object(goal.pose)
        elif goal.operation == PickPlaceGoal.GET_READY:
            self.get_ready()

    def pick_object_id(self, object_id):
        """

        Args:
            object_id:

        @type object_id: int

        Returns:

        """

        for o in self._objects.instances:  # type: ObjInstance
            if o.object_id == object_id:
                obj = PoseStamped()
                obj.pose = o.pose
                obj.header = self._objects.header
                obj.header.stamp = rospy.Time(0)

                transformed_obj = self.tf_listener.transformPose("/base_link", obj)
                pick_pose = transformed_obj.pose

                self._grasped_object = deepcopy(o)
                obj_type = self.get_object_type(o.object_type)
                #pick_pose.position.z += obj_type.bbox.dimensions[obj_type.bbox.BOX_Z] - 0.005
                pick_pose.position.z -= 0.004
                pp = PoseStamped()
                pp.pose = pick_pose
                self.get_ready_for_pick_place()
                self.move_to_pose(pp, 0)
                self.wait_for_final_pose(pp.pose)
                self.set_suction(True)
                pp.pose.position.z = max(pp.pose.position.z + 0.05, 0.1)
                self.move_to_pose(pp, 0)
                self.wait_for_final_pose(pp.pose)
                res = PickPlaceResult()
                res.result = res.SUCCESS
                return res
        res = PickPlaceResult()
        res.result = res.FAILURE
        res.message = "No such object"
        return res

    def place_object(self, place_pose):
        """

        Args:
            place_pose:

        @type place_pose: PoseStamped

        Returns:

        """
        obj_type = self.get_object_type(self._grasped_object.object_type)
        place_pose.pose.position.z = obj_type.bbox.dimensions[obj_type.bbox.BOX_Z]
        #  self.get_ready_for_pick_place()

        transformed_pose = self.tf_listener.transformPose("/base_link", place_pose)
        self.move_to_pose(transformed_pose, 0)
        self.wait_for_final_pose(transformed_pose.pose)
        self.set_suction(False)
        transformed_pose.pose.position.z += 0.03
        self.move_to_pose(transformed_pose)
        self.wait_for_final_pose(transformed_pose.pose)

    def wait_for_final_pose(self, pose, timeout=10):
        end_time = rospy.Time.now() + rospy.Duration(timeout)
        r = rospy.Rate(10)
        while end_time > rospy.Time.now():
            if self.poses_max_diff(self._dobot_pose, pose) < 0.02:

                print("Movement done")
                return
            r.sleep()
        print("Movement failed")
        raise TimeoutException

    def poses_max_diff(self, pose1, pose2):
        """

        Args:
            pose1:
            pose2:

        @type pose1: Pose
        @type pose2: Pose

        Returns:

        @rtype: float

        """
        return float(max(abs(pose1.position.x - pose2.position.x),
                         abs(pose1.position.y - pose2.position.y),
                         abs(pose1.position.z - pose2.position.z)))

    def objects_cb(self, data):
        """

        Args:
            data:

        @type data: InstancesArray

        Returns:

        """
        self._objects = data

    def _init_dobot(self):
        cmd_timeout = SetCmdTimeoutRequest()
        cmd_timeout.timeout = 3000
        self.set_cmd_timeout_srv.call(cmd_timeout)
        self.set_queued_cmd_clear_srv.call(SetQueuedCmdClearRequest())
        self.set_queued_cmd_start_exec_srv.call(SetQueuedCmdStartExecRequest())

        end_effector_params = SetEndEffectorParamsRequest()
        end_effector_params.xBias = 70
        end_effector_params.yBias = 0
        end_effector_params.zBias = 0
        self.set_ent_effector_params_srv.call(end_effector_params)

        ptp_joint_params = SetPTPJointParamsRequest()
        ptp_joint_params.velocity = ptp_joint_params.acceleration = [100, 100, 100, 100]
        self.set_ptp_joint_params_srv.call(ptp_joint_params)

        ptp_coord_params = SetPTPCoordinateParamsRequest()
        ptp_coord_params.xyzAcceleration = 100
        ptp_coord_params.xyzVelocity = 100
        ptp_coord_params.rAcceleration = 100
        ptp_coord_params.rVelocity = 100
        self.set_ptp_coord_params_srv.call(ptp_coord_params)

        ptp_jump_params = SetPTPJumpParamsRequest()
        ptp_jump_params.jumpHeight = 20
        ptp_jump_params.zLimit = 200
        self.set_ptp_jump_params_srv.call(ptp_jump_params)

        ptp_common_params = SetPTPCommonParamsRequest()
        ptp_common_params.velocityRatio = 50
        ptp_common_params.accelerationRatio = 50
        self.set_ptp_common_params_srv.call(ptp_common_params)

        self.set_suction(False)

    def get_pose(self):
        resp = self.get_pose_srv.call(GetPoseRequest())  # type: GetPoseResponse
        self._dobot_pose.position.x = resp.x / 1000.0
        self._dobot_pose.position.y = resp.y / 1000.0
        self._dobot_pose.position.z = resp.z / 1000.0
        self.tf_broadcaster.sendTransform((self._dobot_pose.position.x, self._dobot_pose.position.y,
                                           self._dobot_pose.position.z),
                                          (0, 0, 0, 1), rospy.Time.now(), "suction_cup", "base_link")

    def move_to_cb(self, pose):
        """

        Args:
            pose:

        @type pose: PoseStamped

        Returns:

        """
        self.move_to_pose(pose)

    def move_to_pose(self, pose, mode=1):
        """

        Args:
            pose:
            mode: 1 means direct move to pose, 0 means go up, then above desired pose, then down

        @type pose: PoseStamped
        @type mode: int

        Returns:

        """
        print("Move to ")
        print(pose)
        req = SetPTPCmdRequest()
        req.ptpMode = mode

        req.x = pose.pose.position.x * 1000  # dobot needs position in mm
        req.y = pose.pose.position.y * 1000
        req.z = pose.pose.position.z * 1000
        req.r = 0

        resp = self.move_to_srv.call(req)  # type: SetPTPCmdResponse
        if resp.result == 0:
            return True
        else:
            return False

    def get_suction(self):
        resp = self.get_suction_srv.call(GetEndEffectorSuctionCupRequest())  # type: GetEndEffectorSuctionCupResponse
        return resp.suck

    def set_suction(self, suction):
        """

        Args:
            suction:

        @type suction: bool

        Returns:

        """
        req = SetEndEffectorSuctionCupRequest()
        req.isQueued = True
        req.enableCtrl = 1
        req.suck = 1 if suction else 0
        self.set_suction_srv.call(req)

    def get_object_type(self, object_type):
        """

        Args:
            object_type:

        @type object_type: str

        Returns: getObjectTypeResponse

        """
        req = getObjectTypeRequest()
        req.name = object_type
        resp = self.get_object_type_srv.call(req)  # type: getObjectTypeResponse
        return resp.object_type

    def get_ready_cb(self, req):
        self.get_ready()
        resp = TriggerResponse()
        resp.success = True
        return resp

    def get_ready(self):
        pose = PoseStamped()
        pose.pose.position.x = 0.17
        pose.pose.position.y = 0.0
        pose.pose.position.z = 0.03
        self.move_to_pose(pose)
        self.wait_for_final_pose(pose.pose)

    def get_ready_for_pick_place(self):
        pose = PoseStamped()
        pose.pose.position.x = 0.25
        pose.pose.position.z = 0.1
        self.move_to_pose(pose)
        self.wait_for_final_pose(pose.pose)


if __name__ == '__main__':
    rospy.init_node('art_dobot_arm', log_level=rospy.INFO)

    try:
        node = DobotClient()
        r = rospy.Rate(5)
        while not rospy.is_shutdown():
            node.get_pose()
            r.sleep()
            pass

        node = None
    except rospy.ROSInterruptException:

        node = None
