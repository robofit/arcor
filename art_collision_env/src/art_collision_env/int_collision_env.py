from art_collision_env.collision_env_services import CollisionEnvServices
from interactive_markers.interactive_marker_server import InteractiveMarkerServer
from interactive_markers.menu_handler import MenuHandler
from visualization_msgs.msg import InteractiveMarkerControl, InteractiveMarkerFeedback, InteractiveMarker, Marker
from art_msgs.msg import CollisionPrimitive
from shape_msgs.msg import SolidPrimitive
from art_collision_env.int_funcs import *
from geometry_msgs.msg import PoseStamped


class IntCollisionEnv(CollisionEnvServices):

    def __init__(self, setup, world_frame):

        super(IntCollisionEnv, self).__init__(setup, world_frame)

        self.im_server = InteractiveMarkerServer(self.NS + "markers")

        self.create_menus()

    def create_menus(self):

        self.global_menu_handler = MenuHandler()
        g_art = self.global_menu_handler.insert("Artificial")
        self.global_menu_handler.insert("Add primitive", parent=g_art, callback=self.global_menu_add_prim_cb)
        self.global_menu_handler.insert("Clear all", parent=g_art, callback=self.global_menu_clear_all_cb)
        self.global_menu_handler.insert("Clear all (permanent)", parent=g_art,
                                        callback=self.global_menu_clear_all_perm_cb)
        self.global_menu_handler.insert("Save all", parent=g_art, callback=self.global_menu_save_all_cb)
        self.global_menu_handler.insert("Reload", parent=g_art, callback=self.global_menu_reload_cb)

        g_det = self.global_menu_handler.insert("Detected")
        self.global_menu_handler.insert("Pause", parent=g_det, callback=self.global_menu_pause_cb)
        self.global_menu_handler.insert("Clear all", parent=g_det, callback=self.global_menu_det_clear_all_cb)

        self.a_obj_menu_handler = MenuHandler()
        mov = self.a_obj_menu_handler.insert("Moveable", callback=self.menu_moveable_cb)
        self.a_obj_menu_handler.setCheckState(mov, MenuHandler.UNCHECKED)
        sc = self.a_obj_menu_handler.insert("Scaleable", callback=self.menu_scaleable_cb)
        self.a_obj_menu_handler.setCheckState(sc, MenuHandler.UNCHECKED)
        self.a_obj_menu_handler.insert("Save", callback=self.menu_save_cb)
        self.a_obj_menu_handler.insert("Clear", callback=self.menu_remove_cb)

        self.d_obj_menu_handler = MenuHandler()
        self.d_obj_menu_handler.insert("Clear", callback=self.d_menu_clear_cb)

        int_marker = InteractiveMarker()
        int_marker.header.frame_id = self.world_frame
        int_marker.pose.position.z = 1.2
        int_marker.scale = 0.5
        int_marker.name = "global_menu"
        int_marker.description = "Global Menu"
        control = InteractiveMarkerControl()

        control.interaction_mode = InteractiveMarkerControl.BUTTON
        control.always_visible = True

        marker = Marker()

        marker.type = Marker.CUBE
        marker.scale.x = 0.05
        marker.scale.y = 0.05
        marker.scale.z = 0.05
        marker.color.r = 0.5
        marker.color.g = 0.5
        marker.color.b = 0.5
        marker.color.a = 0.5

        control.markers.append(marker)
        int_marker.controls.append(control)

        self.im_server.insert(int_marker, self.ignored_cb)
        self.global_menu_handler.apply(self.im_server, int_marker.name)
        self.im_server.applyChanges()

    def ignored_cb(self, feedback):

        pass

    def d_menu_clear_cb(self, f):

        pass

    def global_menu_det_clear_all_cb(self, f):

        for name in self.clear_all_det():

            self.im_server.erase(name)

        self.im_server.applyChanges()

    def global_menu_pause_cb(self, f):

        handle = f.menu_entry_id
        state = self.global_menu_handler.getCheckState(handle)

        if state == MenuHandler.CHECKED:

            self.global_menu_handler.setCheckState(handle, MenuHandler.UNCHECKED)
            self.paused = False

        else:

            self.global_menu_handler.setCheckState(handle, MenuHandler.CHECKED)
            self.paused = True

        self.global_menu_handler.reApply(self.im_server)
        self.im_server.applyChanges()

    def global_menu_save_all_cb(self, f):

        self.save_primitives()

    def global_menu_add_prim_cb(self, feedback):

        p = CollisionPrimitive()
        p.name = self._generate_name()
        p.bbox.type = SolidPrimitive.BOX
        p.bbox.dimensions = [0.1, 0.1, 0.1]
        p.pose.header.frame_id = self.world_frame
        p.pose.pose.orientation.w = 1
        p.pose.pose.position.z = 0.5
        p.setup = self.setup

        self.add_primitive(p)

    def global_menu_reload_cb(self, feedback):

        self.reload()

    def global_menu_clear_all_cb(self, feedback):

        self.clear_all(permanent=False)

    def global_menu_clear_all_perm_cb(self, feedback):

        self.clear_all()

    def menu_save_cb(self, f):

        self.save_primitive(f.marker_name)

    def menu_scaleable_cb(self, f):

        handle = f.menu_entry_id
        state = self.a_obj_menu_handler.getCheckState(handle)

        if state == MenuHandler.CHECKED:

            self.a_obj_menu_handler.setCheckState(handle, MenuHandler.UNCHECKED)
            # TODO

        else:

            self.a_obj_menu_handler.setCheckState(handle, MenuHandler.CHECKED)
            # TODO

        self.a_obj_menu_handler.reApply(self.im_server)
        self.im_server.applyChanges()

    def menu_moveable_cb(self, feedback):

        handle = feedback.menu_entry_id
        state = self.a_obj_menu_handler.getCheckState(handle)

        if state == MenuHandler.CHECKED:

            self.a_obj_menu_handler.setCheckState(handle, MenuHandler.UNCHECKED)
            # if feedback.marker_name in self.moveable:
            #    self.moveable.remove(feedback.marker_name)

            self.im_server.erase(feedback.marker_name)
            self.im_server.insert(make_def(self.artificial_objects[feedback.marker_name]), self.process_im_feedback)

        else:

            self.a_obj_menu_handler.setCheckState(handle, MenuHandler.CHECKED)
            self.make_interactive(self.artificial_objects[feedback.marker_name])

        self.a_obj_menu_handler.reApply(self.im_server)
        self.im_server.applyChanges()

    def menu_remove_cb(self, feedback):

        self.remove_name(feedback.marker_name)

    def process_im_feedback(self, feedback):

        if feedback.event_type == InteractiveMarkerFeedback.POSE_UPDATE:

            ps = PoseStamped()
            ps.header = feedback.header
            ps.pose = feedback.pose

            if feedback.marker_name in self.artificial_objects:

                self.set_primitive_pose(feedback.marker_name, ps)

            elif feedback.marker_name:

                # detected objects
                pass

    def make_interactive(self, p):

        im = self.im_server.get(p.name)

        for v in (("x", (1, 0, 0, 1)), ("y", (0, 0, 1, 1)), ("z", (0, 1, 0, 1))):
            im.controls.append(rotate_axis_control("rotate_" + v[0], v[1]))
            im.controls.append(move_axis_control("move_" + v[0], v[1]))

    def remove_name(self, name):

        super(IntCollisionEnv, self).remove_name(name)

        self.im_server.erase(name)
        self.im_server.applyChanges()

    def add_primitive(self, p):

        super(IntCollisionEnv, self).add_primitive(p)

        self.im_server.insert(make_def(p), self.process_im_feedback)
        self.a_obj_menu_handler.apply(self.im_server, p.name)
        self.im_server.applyChanges()

    def reload(self):

        self.im_server.clear()
        self.create_menus()
        super(IntCollisionEnv, self).reload()

    def add_detected(self, name, ps, object_type):

        super(IntCollisionEnv, self).add_detected(name, ps, object_type)

        p = CollisionPrimitive()
        p.name = name
        p.pose = ps
        p.bbox = object_type.bbox

        self.im_server.insert(make_def(p, color=(0, 0, 1)), self.process_im_feedback)
        self.im_server.applyChanges()

    def clear_detected(self, name):

        super(IntCollisionEnv, self).clear_detected(name)
        self.im_server.erase(name)
        self.im_server.applyChanges()
