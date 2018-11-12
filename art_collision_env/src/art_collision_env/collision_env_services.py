import rospy
from std_srvs.srv import Empty as EmptySrv, EmptyResponse
from art_msgs.srv import AddCollisionPrimitive, AddCollisionPrimitiveResponse, StringTrigger, StringTriggerResponse, \
    PoseStampedTrigger, PoseStampedTriggerResponse
from shape_msgs.msg import SolidPrimitive
from art_collision_env.collision_env import CollisionEnv, CollisionEnvException


class CollisionEnvServices(CollisionEnv):

    def __init__(self, setup, world_frame):

        super(CollisionEnvServices, self).__init__(setup, world_frame)

        self.srv_art_clear_all = rospy.Service(self.NS + "artificial/reload", EmptySrv, self.srv_art_reload_cb)
        self.srv_art_clear_all = rospy.Service(self.NS + "artificial/clear/all", EmptySrv, self.srv_art_clear_all_cb)
        self.srv_art_clear_name = rospy.Service(
            self.NS + "artificial/clear/name",
            StringTrigger,
            self.srv_art_clear_name_cb)

        self.srv_art_save = rospy.Service(self.NS + "artificial/save", StringTrigger, self.srv_art_save_cb)
        self.srv_art_save_all = rospy.Service(self.NS + "artificial/save_all", EmptySrv, self.srv_art_save_all_cb)

        self.srv_clear_all = rospy.Service(self.NS + "detected/clear/all", EmptySrv, self.srv_clear_all_cb)
        self.srv_clear_on_table = rospy.Service(
            self.NS + "detected/clear/on_table",
            EmptySrv,
            self.srv_clear_on_table_cb)

        self.srv_clear_except = rospy.Service(self.NS + "detected/clear/except", StringTrigger,
                                              self.srv_clear_except_cb)

        self.srv_clear_out_of_table_except = rospy.Service(self.NS + "detected/out_of_table/clear/except",
                                                           StringTrigger, self.srv_clear_out_of_table_except_cb)

        self.srv_det_set_pose = rospy.Service(self.NS + "detected/set_pose", PoseStampedTrigger,
                                              self.srv_det_set_pose_cb)

        self.srv_pause = rospy.Service(self.NS + "pause", EmptySrv, self.srv_pause_cb)
        self.srv_resume = rospy.Service(self.NS + "resume", EmptySrv, self.srv_resume_cb)

        self.srv_collision_primitive = rospy.Service(self.NS + "artificial/add/primitive", AddCollisionPrimitive,
                                                     self.srv_collision_primitive_cb)

    def srv_art_save_all_cb(self, req):

        self.save_primitives()
        return EmptyResponse()

    def srv_art_save_cb(self, req):

        try:
            self.save_primitive(req.str)
        except CollisionEnvException:
            return StringTriggerResponse(sucess=False)

        return StringTriggerResponse(sucess=True)

    def srv_art_reload_cb(self, req):

        self.reload()
        return EmptyResponse()

    def srv_art_clear_name_cb(self, req):

        return StringTriggerResponse(success=self.remove_name(req.str))

    def srv_collision_primitive_cb(self, req):

        if req.primitive.bbox.type != SolidPrimitive.BOX:
            rospy.logwarn("Only BOX is supported so far.")
            return AddCollisionPrimitiveResponse(name=req.primitive.name, success=False)

        if len(req.primitive.bbox.dimensions) < 3:
            rospy.logwarn("BOX needs three dimensions.")
            return AddCollisionPrimitiveResponse(name=req.primitive.name, success=False)

        if req.primitive.pose.header.frame_id == "":
            rospy.logwarn("Empty frame_id!")
            return AddCollisionPrimitiveResponse(name=req.primitive.name, success=False)

        if req.primitive.name == "":
            req.primitive.name = self._generate_name()

        req.primitive.setup = self.setup

        if req.primitive.name in self.artificial_objects:
            rospy.loginfo("Adding collision primitive: " + req.primitive.name)
        else:
            rospy.loginfo("Updating collision primitive: " + req.primitive.name)

        self.add_primitive(req.primitive)
        self.pub_artificial()

        return AddCollisionPrimitiveResponse(name=req.primitive.name, success=True)

    def srv_clear_except_cb(self, req):

        self.clear_all_det([req.str])
        return StringTriggerResponse(success=True)

    def srv_art_clear_all_cb(self, req):

        self.clear_all(permanent=False)
        return EmptyResponse()

    def srv_det_set_pose_cb(self, req):

        if not self.paused:
            return PoseStampedTriggerResponse(success=False, message="Call pause first.")

        if req.ps.header.frame_id != self.oh.header.frame_id:
            return PoseStampedTriggerResponse(success=False, message="Invalid frame_id.")

        if req.str not in self.oh.objects:
            return PoseStampedTriggerResponse(success=False, message="Unknown object.")

        self.set_det_pose(req.str, req.ps)

        return PoseStampedTriggerResponse(success=True)

    def srv_clear_out_of_table_except_cb(self, req):

        cnt = len(self.clear_det_on_table(inv=True, ignore=[req.str]))
        rospy.loginfo("Cleared " + str(cnt) + " detected objects out of table, with exception of " + req.str + ".")
        return StringTriggerResponse(success=True)

    def srv_clear_on_table_cb(self, req):

        cnt = len(self.clear_det_on_table())
        rospy.loginfo("Cleared " + str(cnt) + " detected objects on table.")
        return EmptyResponse()

    def srv_clear_all_cb(self, req):

        self.clear_all_det()
        return EmptyResponse()

    def srv_pause_cb(self, req):

        rospy.loginfo("Paused")
        self.paused = True
        return EmptyResponse()

    def srv_resume_cb(self, req):

        rospy.loginfo("Resumed")
        self.paused = False
        return EmptyResponse()

    def srv_clear_cb(self, req):

        return EmptyResponse()
