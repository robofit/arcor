import rospy
from moveit_commander import PlanningSceneInterface
from art_utils import ObjectHelper, ArtApiHelper
from geometry_msgs.msg import PoseStamped
from std_srvs.srv import Empty as EmptySrv, EmptyResponse
from std_msgs.msg import Bool
from art_msgs.srv import AddCollisionBox, AddCollisionBoxResponse, StringTrigger, StringTriggerResponse
from art_msgs.msg import CollisionObjects
import uuid
from threading import RLock

"""
TODO
service to set object pose (usefull in paused state)
"""


class CollisionEnv(object):

    DETECTED_OBJECT_PREFIX = "dobj_"

    def __init__(self):

        self.lock = RLock()

        self.ps = PlanningSceneInterface()
        self.api = ArtApiHelper()
        self._paused = False
        self.oh = ObjectHelper(self.object_cb)
        self.artificial_objects = {}

        self.collision_objects_pub = rospy.Publisher("artificial", CollisionObjects, latch=True)
        self.pub_artificial()

        self.timer = rospy.Timer(1.0, self.timer)

        self.srv_art_clear_all = rospy.Service("artificial/clear/all", EmptySrv, self.srv_art_clear_all_cb)
        self.srv_art_clear_name = rospy.Service("artificial/clear/name", StringTrigger, self.srv_art_clear_name_cb)

        self.srv_clear_all = rospy.Service("detected/clear/all", EmptySrv, self.srv_clear_all_cb)
        self.srv_clear_on_table = rospy.Service("detected/clear/on_table", EmptySrv, self.srv_clear_on_table_cb)

        self.paused_pub = rospy.Publisher("paused", Bool, latch=True)
        self.paused = False
        self.srv_pause = rospy.Service("pause", EmptySrv, self.srv_pause_cb)
        self.srv_resume = rospy.Service("resume", EmptySrv, self.srv_resume_cb)

        self.srv_collision_box = rospy.Service("collision_box", AddCollisionBox, self.srv_collision_box_cb)

    def pub_artificial(self):

        msg = CollisionObjects()

        for k, v in self.artificial_objects:

            msg.boxes.append(v)

        self.collision_objects_pub.publish(msg)

    @property
    def paused(self):

        return self._paused

    @paused.setter
    def paused(self, val):

        self._paused = val
        self.paused_pub.publish(val)

    def srv_art_clear_name_cb(self, req):

        with self.lock:

            if req.str not in self.artificial_objects:
                return StringTriggerResponse(success=False, message="Unknown artificial object")

            self.ps.remove_world_object(req.str)
            del self.artificial_objects[req.str]
            self.pub_artificial()

    def srv_collision_box_cb(self, req):

        with self.lock:

            if req.cb.name == "":
                req.cb.name = str(uuid.uuid4())

            self.artificial_objects[req.cb.name] = req.cb
            self.ps.add_box(req.cb.name, req.cb.pose, req.cb.bbox.dimensions)
            self.pub_artificial()

            return AddCollisionBoxResponse(name=req.name, success=True)

    def srv_art_clear_all_cb(self, req):

        with self.lock:

            for k, v in self.artificial_objects:

                self.ps.remove_world_object(k)

            self.artificial_objects = {}
            self.pub_artificial()

    def srv_clear_on_table_cb(self, req):

        with self.lock:

            for k, v in self.oh.objects:

                if not v.on_table:
                    return

                self.ps.remove_world_object(self.DETECTED_OBJECT_PREFIX + v.object_id)

            return EmptyResponse()

    def srv_clear_all_cb(self, req):

        with self.lock:

            for k, v in self.oh.objects:
                self.ps.remove_world_object(self.DETECTED_OBJECT_PREFIX + v.object_id)

            return EmptyResponse()

    def srv_pause_cb(self, req):

        self.paused = True
        return EmptyResponse()

    def srv_resume_cb(self, req):

        self.paused = False
        return EmptyResponse()

    def srv_clear_cb(self, req):

        return EmptyResponse()

    def timer(self):

        if self.paused:
            return

        with self.lock:

            known_objects = self.ps.get_known_object_names()

            for name in known_objects:

                if name.startswith(self.DETECTED_OBJECT_PREFIX) and name not in self.oh.objects:

                    self.ps.remove_world_object(name)

            # restore artificial objects if they are lost somehow (e.g. by restart  of MoveIt!)
            for k, v in self.artificial_objects:

                if k in known_objects:
                    continue

                self.ps.add_box(v.name, v.pose, v.bbox.dimensions)

    def object_cb(self, evt, h, inst):

        if self.paused:
            return

        with self.lock:

            attached_objects = self.ps.get_attached_objects()

            if evt in (ObjectHelper.OBJECT_ADDED, ObjectHelper.OBJECT_UPDATED):

                if inst.object_id in attached_objects:
                    return

                ps = PoseStamped()
                ps.header = h
                ps.pose = inst.pose

                object_type = self.api.get_object_type(inst.object_type)

                if object_type is not None:

                    self.ps.add_box(self.DETECTED_OBJECT_PREFIX + inst.object_id, ps, object_type.bbox.dimensions)

            elif evt == ObjectHelper.OBJECT_LOST:

                if inst.object_id not in attached_objects:

                    self.ps.remove_world_object(self.DETECTED_OBJECT_PREFIX + inst.object_id)
