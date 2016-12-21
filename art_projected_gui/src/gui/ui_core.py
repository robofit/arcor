#!/usr/bin/env python

from PyQt4 import QtGui, QtCore
from items import ObjectItem,  PlaceItem,  LabelItem,  ProgramItem,  PolygonItem, SquareItem
import rospy

class customGraphicsView(QtGui.QGraphicsView):

    def __init__(self, parent=None):
        QtGui.QGraphicsView.__init__(self, parent)

        self.setHorizontalScrollBarPolicy(QtCore.Qt.ScrollBarAlwaysOff)
        self.setVerticalScrollBarPolicy(QtCore.Qt.ScrollBarAlwaysOff)

    def resizeEvent(self, evt=None):

        self.fitInView(self.sceneRect(),  QtCore.Qt.KeepAspectRatio)

class UICore(QtCore.QObject):
    """Class holds QGraphicsScene and its content (items).

    There are methods for manipulation (add, find, delete) with items.
    There should not be any ROS-related stuff, just basic things.
    All items to be displayed (objects, places etc.) are inserted into array with few exceptions (e.g. program visualization, notifications).

    Attributes:
        x (float): x coordinate of the scene's origin (in world coordinate system, meters).
        y (float): dtto.
        width (float): Width of the scene.
        height (float): dtto
        rpm (int): Resolution per meter (pixels per meter of width/height).
        scene (QGraphicsScene): Holds all Item(s), manages (re)painting etc.
        bottom_label (LabelItem): Label for displaying messages to user.
        program_vis (ProgramItem): Item to display robot's program.
        scene_items (list): Array to hold all displayed items.
        view (QGraphicsView): To show content of the scene in debug window.
    """

    def __init__(self,  x,  y,  width,  height,  rpm):
        """
        Args:
            x (float): x coordinate of the scene's origin (in world coordinate system, meters).
            y (float): dtto.
            width (float): Width of the scene.
            height (float): dtto
            rpm (int): Resolution per meter (pixels per meter of width/height).
        """

        super(UICore, self).__init__()

        self.x = x
        self.y = y
        self.width = width
        self.height = height
        self.rpm = rpm

        w = self.width*self.rpm
        h = self.height/self.width*w

        self.scene = QtGui.QGraphicsScene(0, 0,  int(w),  int(h))
        self.scene.setBackgroundBrush(QtCore.Qt.black)
        #self.scene.setItemIndexMethod(QtGui.QGraphicsScene.NoIndex) # should be good for dynamic scenes

        self.bottom_label = LabelItem(self.scene,  self.rpm,  0.1,  self.height - 0.05,  self.width-0.2,  0.1)
        self.program_vis = ProgramItem(self.scene,  self.rpm,  0.1,  0.1)

        self.scene_items = []

        self.view = customGraphicsView(self.scene)
        self.view.setRenderHint(QtGui.QPainter.Antialiasing)
        self.view.setViewportUpdateMode(QtGui.QGraphicsView.SmartViewportUpdate)
        self.view.setStyleSheet( "QGraphicsView { border-style: none; }" )

    def notif(self,  msg,  min_duration=3.0,  temp = False):
        """Display message (notification) to the user.

        Args:
            msg (str): Message to be displayed.
            min_duration (:obj:`float`, optional): Message should be displayed for min_duration seconds at least.
            temp (:obj:`bool`, optional): temporal message disappears after min_duration and last non-temporal message is displayed instead.
        """

        self.bottom_label.add_msg(msg,  rospy.Duration(min_duration),  temp)

    def debug_view(self):
        """Show window with scene - for debugging purposes."""

        self.view.show()

    def get_scene_items_by_type(self,  type):
        """Generator to filter content of scene_items array."""

        for el in self.scene_items:
           if isinstance(el,  type): yield el

    def remove_scene_items_by_type(self,  type):
        """Removes items of the given type from scene (from scene_items and scene)."""

        its = []

        for it in self.scene_items:

            if not isinstance(it, type): continue
            its.append(it)

        for it in its:

            self.scene.removeItem(it)
            self.scene_items.remove(it)

    def add_object(self,  object_id,  object_type,  x,  y,  sel_cb=None):
        """Adds object to the scene.

        Args:
            object_id (str): unique object ID
            object_type (str): type (category) of the object
            x, y (float): world coordinates
            sel_cb (method): Callback which gets called one the object is selected.
        """

        self.scene_items.append(ObjectItem(self.scene,  self.rpm, object_id,  object_type,  x,  y,  sel_cb))

    def remove_object(self,  object_id):
        """Removes ObjectItem with given object_id from the scene."""

        obj = None

        for it in self.get_scene_items_by_type(ObjectItem):

            if it.object_id == object_id:

                obj = it
                break

        if obj is not None:

            self.scene.removeItem(obj)
            self.scene_items.remove(obj)
            return True

        return False

    def select_object(self,  obj_id,  unselect_others=True):
        """Sets ObjectItem with given obj_id as selected. By default, all other items are unselected."""

        if obj_id == "": return

        for it in self.get_scene_items_by_type(ObjectItem):

            if it.object_id == obj_id:

                it.set_selected(True)
                if not unselect_others: break

            elif unselect_others:

                it.set_selected(False)

    def select_object_type(self,  obj_type,  unselect_others=True):
        """Sets all ObjectItems with geiven object_type and selected. By default, all objects of other types are unselected."""

        if obj_type == "": return

        for it in self.get_scene_items_by_type(ObjectItem):

            if it.object_type == obj_type:
                it.set_selected(True)
            elif unselect_others: it.set_selected(False)

    def get_object(self,  obj_id):
        """Returns ObjectItem with given object_id or None if the ID is not found."""

        for it in self.get_scene_items_by_type(ObjectItem):

            if it.object_id == obj_id: return it

        return None

    def add_place(self,  caption,  x,  y,  place_cb=None,  fixed=False):

        self.scene_items.append(PlaceItem(self.scene,  self.rpm,  caption,  x,  y,  place_pose_changed=place_cb,  fixed=fixed))

    def add_polygon(self,  caption,  obj_coords=[],  poly_points=[],  polygon_changed=None):

        self.scene_items.append(PolygonItem(self.scene,  self.rpm,  caption,  obj_coords,  poly_points, polygon_changed))

    def add_square(self, caption, min_x, min_y, square_width, square_height, square_changed=None):

        self.scene_items.append(SquareItem(self.scene, self.rpm, caption, min_x, min_y, square_width, square_height, square_changed))

    def clear_places(self):

        self.remove_scene_items_by_type(PlaceItem)

    def clear_all(self):

        for it in self.get_scene_items_by_type(ObjectItem):

            it.set_selected(False)

        self.remove_scene_items_by_type(PlaceItem)
        self.remove_scene_items_by_type(PolygonItem)
        self.remove_scene_items_by_type(SquareItem)
