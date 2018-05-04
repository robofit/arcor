import rospy
from moveit_commander import PlanningSceneInterface
from art_utils import ObjectHelper, ArtApiHelper
from geometry_msgs.msg import PoseStamped
from std_srvs.srv import Empty as EmptySrv, EmptyResponse
from std_msgs.msg import Bool
from art_msgs.srv import AddCollisionPrimitive, AddCollisionPrimitiveResponse, StringTrigger, StringTriggerResponse
from art_msgs.msg import CollisionObjects
import uuid
from threading import RLock
from shape_msgs.msg import SolidPrimitive
from interactive_markers.interactive_marker_server import InteractiveMarkerServer
from visualization_msgs.msg import InteractiveMarkerControl, InteractiveMarkerFeedback, InteractiveMarker, Marker


"""
TODO
tests!!!
service to set detected object pose - usefull in paused state (fake pose)
grouping of artificial objects - to move whole group at once
ability to set/store/update artificial objects wrt the program block/instruction?
"""


class CollisionEnv(object):

    DETECTED_OBJECT_PREFIX = "dobj_"

    def __init__(self, setup):

        assert setup != ""

        self.setup = setup

        self.api = ArtApiHelper()
        rospy.loginfo("Waiting for DB API")
        self.api.wait_for_db_api()

        self.lock = RLock()

        self.ps = PlanningSceneInterface()

        self._paused = False
        self.oh = ObjectHelper(self.object_cb)
        self.artificial_objects = {}

        self.im_server = InteractiveMarkerServer("art_collision_env")

        for prim in self.api.get_collision_primitives(self.setup):

            rospy.loginfo("Loading object: " + prim.name)
            self._add_primitive(prim)

        self.collision_objects_pub = rospy.Publisher("artificial", CollisionObjects, latch=True, queue_size=1)
        self.pub_artificial()

        self.timer = rospy.Timer(rospy.Duration(1.0), self.timer_cb)

        self.srv_art_clear_all = rospy.Service("artificial/clear/all", EmptySrv, self.srv_art_clear_all_cb)
        self.srv_art_clear_name = rospy.Service("artificial/clear/name", StringTrigger, self.srv_art_clear_name_cb)

        self.srv_clear_all = rospy.Service("detected/clear/all", EmptySrv, self.srv_clear_all_cb)
        self.srv_clear_on_table = rospy.Service("detected/clear/on_table", EmptySrv, self.srv_clear_on_table_cb)

        self.paused_pub = rospy.Publisher("paused", Bool, latch=True, queue_size=1)
        self.paused = False
        self.srv_pause = rospy.Service("pause", EmptySrv, self.srv_pause_cb)
        self.srv_resume = rospy.Service("resume", EmptySrv, self.srv_resume_cb)

        self.srv_collision_primitive = rospy.Service("artificial/add/primitive", AddCollisionPrimitive,
                                                     self.srv_collision_primitive_cb)

        rospy.loginfo("Ready")

    def process_im_feedback(self, feedback):

        if feedback.event_type == InteractiveMarkerFeedback.POSE_UPDATE:
            rospy.loginfo(feedback.marker_name + ": pose changed")

    def _add_primitive(self, p):

        self.artificial_objects[p.name] = p
        self.ps.add_box(p.name, p.pose, p.bbox.dimensions)

        im = InteractiveMarker()
        im.header.frame_id = p.pose.header.frame_id
        im.pose = p.pose.pose
        im.name = p.name
        im.description = "Artificial collision object"
        im.scale = 1

        marker = Marker()

        marker.type = Marker.CUBE
        marker.scale.x = p.bbox.dimensions[0]
        marker.scale.y = p.bbox.dimensions[1]
        marker.scale.z = p.bbox.dimensions[2]
        marker.color.r = 0.5
        marker.color.g = 0.5
        marker.color.b = 0.5
        marker.color.a = 1.0

        control = InteractiveMarkerControl()
        control.always_visible = True
        control.markers.append(marker)
        im.controls.append(control)

        crx = InteractiveMarkerControl()
        crx.orientation.w = 1
        crx.orientation.x = 1
        crx.orientation.y = 0
        crx.orientation.z = 0
        crx.name = "rotate_x"
        crx.interaction_mode = InteractiveMarkerControl.ROTATE_AXIS
        im.controls.append(crx)

        cry = InteractiveMarkerControl()
        cry.orientation.w = 1
        cry.orientation.x = 0
        cry.orientation.y = 0
        cry.orientation.z = 1
        cry.name = "rotate_y"
        cry.interaction_mode = InteractiveMarkerControl.ROTATE_AXIS
        im.controls.append(cry)

        crz = InteractiveMarkerControl()
        crz.orientation.w = 1
        crz.orientation.x = 0
        crz.orientation.y = 1
        crz.orientation.z = 0
        crz.name = "rotate_z"
        crz.interaction_mode = InteractiveMarkerControl.ROTATE_AXIS
        im.controls.append(crz)

        cpx = InteractiveMarkerControl()
        cpx.orientation.w = 1
        cpx.orientation.x = 1
        cpx.orientation.y = 0
        cpx.orientation.z = 0
        cpx.name = "move_x"
        cpx.interaction_mode = InteractiveMarkerControl.MOVE_AXIS
        im.controls.append(cpx)

        cpy = InteractiveMarkerControl()
        cpy.orientation.w = 1
        cpy.orientation.x = 0
        cpy.orientation.y = 0
        cpy.orientation.z = 1
        cpy.name = "move_y"
        cpy.interaction_mode = InteractiveMarkerControl.MOVE_AXIS
        im.controls.append(cpy)

        cpz = InteractiveMarkerControl()
        cpz.orientation.w = 1
        cpz.orientation.x = 0
        cpz.orientation.y = 1
        cpz.orientation.z = 0
        cpz.name = "move_z"
        cpz.interaction_mode = InteractiveMarkerControl.MOVE_AXIS
        im.controls.append(cpz)

        self.im_server.insert(im, self.process_im_feedback)
        self.im_server.applyChanges()

    def pub_artificial(self):

        msg = CollisionObjects()

        for k, v in self.artificial_objects.iteritems():

            msg.primitives.append(v)

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
                msg = "Unknown artificial object: " + req.str
                rospy.logwarn(msg)
                return StringTriggerResponse(success=False, message=msg)

            self.ps.remove_world_object(req.str)
            del self.artificial_objects[req.str]
            self.pub_artificial()

        if not self.api.clear_collision_primitives(self.setup, names=[req.str]):
            rospy.logwarn("Failed to remove from permanent storage")

        rospy.loginfo("Removed object: " + req.str)
        return StringTriggerResponse(success=True)

    def srv_collision_primitive_cb(self, req):

        with self.lock:

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
                req.primitive.name = str(uuid.uuid4())

            req.primitive.setup = self.setup

            if req.primitive.name in self.artificial_objects:
                rospy.loginfo("Adding collision primitive: " + req.primitive.name)
            else:
                rospy.loginfo("Updating collision primitive: " + req.primitive.name)

            self._add_primitive(req.primitive)
            self.pub_artificial()

        if not self.api.add_collision_primitive(req.primitive):
            self.logwarn("Failed to save to permanent storage.")

        return AddCollisionPrimitiveResponse(name=req.primitive.name, success=True)

    def srv_art_clear_all_cb(self, req):

        with self.lock:

            rospy.loginfo("Clearing " + str(len(self.artificial_objects)) + " artificial objects...")

            for k, v in self.artificial_objects.iteritems():

                self.ps.remove_world_object(k)

            self.artificial_objects = {}
            self.pub_artificial()

        if not self.api.clear_collision_primitives(self.setup):
            rospy.logwarn("Failed to remove from permanent storage")

        return EmptyResponse()

    def srv_clear_on_table_cb(self, req):

        cnt = 0

        with self.lock:

            for k, v in self.oh.objects:

                if not v.on_table:
                    continue

                cnt += 1
                self.ps.remove_world_object(self.DETECTED_OBJECT_PREFIX + v.object_id)

        rospy.loginfo("Cleared " + str(cnt) + " detected objects on table.")
        return EmptyResponse()

    def srv_clear_all_cb(self, req):

        with self.lock:

            rospy.loginfo("Clearing all (" + str(len(self.oh.objects)) + ") detected objects.")

            for k, v in self.oh.objects:
                self.ps.remove_world_object(self.DETECTED_OBJECT_PREFIX + v.object_id)

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

    def timer_cb(self, evt):

        if self.paused:
            return

        with self.lock:

            known_objects = self.ps.get_known_object_names()

            for name in known_objects:

                if name.startswith(self.DETECTED_OBJECT_PREFIX) and name not in self.oh.objects:

                    rospy.loginfo("Removing outdated detected object: " + name)
                    self.ps.remove_world_object(name)

            # restore artificial objects if they are lost somehow (e.g. by restart  of MoveIt!)
            for k, v in self.artificial_objects.iteritems():

                if k in known_objects:
                    continue

                rospy.loginfo("Restoring artificial object: " + v.name)
                if v.bbox.type == SolidPrimitive.BOX:
                    self.ps.add_box(v.name, v.pose, v.bbox.dimensions)
                else:
                    # TODO other types
                    pass

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
