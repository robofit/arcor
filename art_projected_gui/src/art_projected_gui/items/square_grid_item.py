#!/usr/bin/env python

from PyQt4 import QtGui, QtCore
from item import Item
from art_projected_gui.items import PlaceItem, ButtonItem, ObjectItem
from desc_item import DescItem
from dialog_item import DialogItem
from math import modf
from geometry_msgs.msg import PoseStamped
# from art_msgs.msg import ObjectType
# from shape_msgs.msg import SolidPrimitive
from art_projected_gui.helpers import conversions

translate = QtCore.QCoreApplication.translate

'''
    Class depicts grid corners. If some corner is moved, class informs its parent.
'''


class SquarePointItem(Item):

    # Class constructor
    def __init__(self, scene, x, y, parent, corner, fixed, changed=False):

        self.outline_diameter = 0.025

        super(SquarePointItem, self).__init__(scene, x, y, 0, parent)

        self.corner = corner
        self.changed = changed

        self.fixed = fixed

    '''
        Method returns type of the corner (e.g. BR = bottom-right)
    '''

    def get_corner(self):

        return self.corner

    '''
        Method sets an attribute "changed".
    '''

    def set_changed(self, changed):

        self.changed = changed

    '''
        Method returns an attribute "changed".
    '''

    def get_changed(self):

        return self.changed

    '''
        Method defines the outer bounds of the item as a rectangle.
    '''

    def boundingRect(self):

        es = self.m2pix(self.outline_diameter * 1.8)

        return QtCore.QRectF(-es / 2, -es / 2, es, es)

    '''
         Method returns the shape of this corner as a QPainterPath in local coordinates.
    '''

    def shape(self):

        path = QtGui.QPainterPath()
        es = self.m2pix(self.outline_diameter)
        path.addEllipse(QtCore.QPoint(0, 0), es / 2, es / 2)
        return path

    '''
        Metoda nastavuje atribut changed a vola rodicovsku metodu point_changed(), ked sa pohne s rohom.
        Method sets an attribute "changed" and calls parent's method "point_changed" when some corner is moved.
    '''

    def item_moved(self):

        self.changed = True
        self.parentItem().point_changed()

    '''
        Metoda vola rodicovsku metodu point_changed, ked sa pusti roh.
        Method calls parent's method "point_changed" when a corner is released.
    '''

    def cursor_release(self):

        self.parentItem().point_changed(True)

    '''
        Method paints the corner.
    '''

    def paint(self, painter, option, widget):

        painter.setClipRect(option.exposedRect)
        painter.setRenderHint(QtGui.QPainter.Antialiasing)

        es = self.m2pix(self.outline_diameter)

        if self.hover and not self.fixed:

            painter.setBrush(QtCore.Qt.gray)
            painter.setPen(QtCore.Qt.gray)
            painter.drawEllipse(QtCore.QPoint(0, 0), es / 2 * 1.8, es / 2 * 1.8)

        painter.setBrush(QtCore.Qt.cyan)
        painter.setPen(QtCore.Qt.cyan)

        painter.drawEllipse(QtCore.QPoint(0, 0), es / 2, es / 2)


'''
    Class draws a grid, objects in the grid, buttons and a text below this grid.
    It implements grid's functionality such as increasing/decreasing grid proportions, rotation of objects in the grid,
    changing spacing between objects and even distribution of objects in this grid.
'''


class SquareItem(Item):

    # Class constructor
    def __init__(self, scene, caption, min_x, min_y, square_width, square_height, object_type, poses, grid_points, scene_items, square_changed=None, fixed=False):

        self.scn = scene
        self.caption = caption
        self.object_type = object_type

        self.scene_items = scene_items
        self.square_changed = square_changed
        self.space = 0.05   # is added to bbox of an object

        self.object_side_length_x = self.object_type.bbox.dimensions[0] + self.space
        self.object_side_length_y = self.object_type.bbox.dimensions[1] + self.space
        self.poses = poses

        self.square = QtGui.QPolygon()

        self.previous_width = 0
        self.previous_height = 0

        self.items = []
        self.last_corner = "BR"

        if len(grid_points) == 0:
            self.min = [min_x, min_y]
            self.max = [min_x + square_width, min_y + square_height]
            self.pom_min = self.min     # to save original value because y is changed in update_bound
            self.pom_max = self.max
        else:
            self.min = list(min(grid_points))
            self.max = list(max(grid_points))

        self.orig_x = []
        self.orig_y = []

        self.desc = None
        self.dialog = None
        self.horizontal = False

        super(SquareItem, self).__init__(scene, min_x, min_y)
        self.fixed = fixed

        self.desc = DescItem(scene, self.min[0] - 0.01, self.min[1] - 0.015, self)
        self.desc.setFlag(QtGui.QGraphicsItem.ItemIgnoresTransformations)
        self.update_text()

        self.plus_btn = ButtonItem(scene, self.min[0] - 0.01, self.min[1] - 0.04, "+", self, self.plus_clicked, 1.25,
                                   QtCore.Qt.darkCyan, width=0.04)
        self.minus_btn = ButtonItem(scene, self.min[0] + 0.035, self.min[1] - 0.04, "-", self, self.minus_clicked, 1.25,
                                    QtCore.Qt.darkCyan, width=0.04)
        self.plus_btn.setEnabled(False)
        self.minus_btn.setEnabled(False)

        self.pts = []
        self.pts.append(SquarePointItem(scene, self.min[0], self.min[1], self, "BL", self.fixed))  # bottom-left corner
        self.pts.append(SquarePointItem(scene, self.max[0], self.min[1], self, "BR", self.fixed))  # bottom-right corner
        self.pts.append(SquarePointItem(scene, self.max[0], self.max[1], self, "TR", self.fixed))  # top-right corner
        self.pts.append(SquarePointItem(scene, self.min[0], self.max[1], self, "TL", self.fixed))  # top-left corner

        if len(poses) > 0 and self.fixed:
            # depicts fixed objects
            for pose in poses:
                it = PlaceItem(
                    self.scn,
                    "Object",
                    pose.pose.position.x,
                    pose.pose.position.y,
                    pose.pose.position.z,
                    conversions.q2a(pose.pose.orientation),
                    self.object_type,
                    None,
                    place_pose_changed=None,
                    fixed=True,
                    txt=False,
                    parent=self,
                    horizontal=self.horizontal
                )
                self.items.append(it)
        else:
            # depicts editable objects
            for i, pose in enumerate(poses):
                rot_point = None  # to save xy coordinates for rotation point (for rotating objects in a grid)
                if i == 0:
                    rot = True
                    pom = self.find_corner("TL").get_pos()
                    rot_point = [pom[0] - 0.025, pom[1] + 0.025]
                else:
                    rot = False
                it = PlaceItem(
                    self.scn,
                    "Object",
                    pose.pose.position.x,
                    pose.pose.position.y,
                    pose.pose.position.z,
                    conversions.q2a(pose.pose.orientation),
                    self.object_type,
                    None,
                    place_pose_changed=None,
                    fixed=False,
                    txt=False,
                    rot=rot,
                    rot_point=rot_point,
                    rotation_changed=self.items_rotation_changed,
                    parent=self,
                    horizontal=self.horizontal
                )
                it.update_point()
                self.items.append(it)

            if self.items:
                self.items[0].set_other_items(self.items[1:])   # rotation of all objects in a grid (list of all objects is handed over to the first object. When the first object is rotated, all objects are rotated)
                self.plus_btn.setEnabled(True)
                self.minus_btn.setEnabled(True)
                self.dialog = DialogItem(self.scene(),
                                         self.pix2m(self.scene().width() / 2),
                                         0.1,
                                         translate(
                    "Place item",
                    "Object place pose options"),
                    [
                    translate(
                        "Place item", "Rotate |"),
                    translate(
                        "Place item", "Rotate --")
                ],
                    self.dialog_cb)

        if self.square_changed is not None:
            self.square_changed(self.get_square_points(), self.items)   # to save grid points and objects into the ProgramItem message

        self.update_bound()
        self.update()

    '''
        Method updates text below the grid.
    '''

    def update_text(self):

        if self.desc is None:
            return

        desc = self.caption
        self.desc.set_content(desc)

    '''
        Method is called when "+" button is pushed. It increase spacing between objects and walls of the box,
        and between objects.
    '''

    def plus_clicked(self, btn):
        self.space += 0.01
        self.point_changed(True, self.last_corner)

    '''
        Method is called when "-" button is pushed. It decrease spacing between objects and walls of the box,
        and between objects.
    '''

    def minus_clicked(self, btn):
        if self.space > 0.02:
            self.space -= 0.01
            self.point_changed(True, self.last_corner)

    '''
        Method updates the bounding rectangle.
    '''

    def update_bound(self):

        self.min[0] = self.pix2m(self.pts[0].x())
        self.min[1] = self.pix2m(self.pts[0].y())
        self.max[0] = self.pix2m(self.pts[0].x())
        self.max[1] = self.pix2m(self.pts[0].y())

        self.orig_x = []
        self.orig_y = []
        for pt in self.pts:

            p = (self.pix2m(pt.x()), self.pix2m(pt.y()))

            if p[0] < self.min[0]:
                self.min[0] = p[0]
            if p[1] < self.min[1]:
                self.min[1] = p[1]

            if p[0] > self.max[0]:
                self.max[0] = p[0]
            if p[1] > self.max[1]:
                self.max[1] = p[1]

            point = pt.get_pos()
            self.orig_x.append(point[0])
            self.orig_y.append(point[1])

        self.pom_min = [min(self.orig_x), min(self.orig_y)]
        self.pom_max = [max(self.orig_x), max(self.orig_y)]

    '''
        Method returns a required corner.
    '''

    def find_corner(self, corner):
        for pt in self.pts:
            if pt.get_corner() == corner:
                return pt
        return None

    '''
        Metoda pre vykreslovanie gridu a objektov v nom. Je volana vzdy, ked sa pohne s niektorym rohom.
        Rovnomerne rozmiestnuje objekty v gride, kontroluje ci nie su v kolizii.
        Zaistuje ukladanie poloh bodov a gridu do spravy ProgramItem.

        Method depics the gird and objects in it. It is called, when some corner is moved.
        It secures even distribution of objects in the grid, checks collisions.
        Also secures saving grid points and positions of objects into the ProgramItem message.
    '''

    def point_changed(self, finished=False, corner=""):

        if self.fixed:
            return

        self.prepareGeometryChange()

        corner = corner

        # update of bounding rect
        self.update_bound()
        for pt in self.pts:
            if (pt.get_corner() == "BR") and pt.get_changed():
                self.find_corner("TR").setPos(pt.x(), self.find_corner("TR").y())
                self.find_corner("BL").setPos(self.find_corner("BL").x(), pt.y())
                self.desc.setPos(self.m2pix(self.min[0] - 0.01), self.m2pix(self.max[1] + 0.015))
                corner = "BR"
                pt.set_changed(False)
            elif (pt.get_corner() == "BL") and pt.get_changed():
                self.find_corner("TL").setPos(pt.x(), self.find_corner("TL").y())
                self.find_corner("BR").setPos(self.find_corner("BR").x(), pt.y())
                self.desc.setPos(self.m2pix(self.min[0] - 0.01), self.m2pix(self.max[1] + 0.015))
                corner = "BL"
                pt.set_changed(False)
            elif (pt.get_corner() == "TL") and pt.get_changed():
                self.find_corner("BL").setPos(pt.x(), self.find_corner("BL").y())
                self.find_corner("TR").setPos(self.find_corner("TR").x(), pt.y())
                self.desc.setPos(self.m2pix(self.min[0] - 0.01), self.m2pix(self.max[1] + 0.015))
                corner = "TL"
                pt.set_changed(False)
            elif (pt.get_corner() == "TR") and pt.get_changed():
                self.find_corner("BR").setPos(pt.x(), self.find_corner("BR").y())
                self.find_corner("TL").setPos(self.find_corner("TL").x(), pt.y())
                self.desc.setPos(self.m2pix(self.min[0] - 0.01), self.m2pix(self.max[1] + 0.015))
                corner = "TR"
                pt.set_changed(False)
            self.plus_btn.setPos(self.m2pix(self.min[0] - 0.01), self.m2pix(self.max[1] + 0.04))
            self.minus_btn.setPos(self.m2pix(self.min[0] + 0.035), self.m2pix(self.max[1] + 0.04))

        if corner != "":

            self.object_side_length_x = self.object_type.bbox.dimensions[0] + self.space
            if not self.horizontal:
                self.object_side_length_y = self.object_type.bbox.dimensions[1] + self.space
            else:
                self.object_side_length_y = self.object_type.bbox.dimensions[2] + self.space

            width_count = int(modf(round((((self.max[0] - self.min[0]) - self.space) / self.object_side_length_x), 5))[1])
            height_count = int(modf(round((((self.max[1] - self.min[1]) - self.space) / self.object_side_length_y), 5))[1])
            if self.previous_width != width_count or self.previous_height != height_count:
                ps = PoseStamped()
                if corner == "BR" or corner == "TR":
                    ps.pose.position.x = self.pom_min[0] + self.space / 2 + self.object_side_length_x / 2
                else:
                    ps.pose.position.x = self.pom_max[0] - self.space / 2 - self.object_side_length_x / 2

                if corner == "BR" or corner == "BL":
                    ps.pose.position.y = self.pom_max[1] - self.space / 2 - self.object_side_length_y / 2
                else:
                    ps.pose.position.y = self.pom_min[1] + self.space / 2 + self.object_side_length_y / 2
                ps.pose.orientation.w = 1.0

                if self.items:
                    rotation = self.items[0].rotation()
                else:
                    rotation = 0.0

                for it in self.items:
                    self.scn.removeItem(it)
                del self.items[:]

                for i in range(0, height_count):
                    for j in range(0, width_count):
                        rot_point = None    # to save xy coordinates for rotation point (for rotating objects in a grid)
                        if corner == "BR" and i == 0 and j == 0:
                            rot = True
                            pom = self.find_corner("TL").get_pos()
                            rot_point = [pom[0] - 0.025, pom[1] + 0.025]
                        elif corner == "BL" and i == 0 and j == 0:
                            rot = True
                            pom = self.find_corner("TR").get_pos()
                            rot_point = [pom[0] + 0.025, pom[1] + 0.025]
                        elif corner == "TR" and i == 0 and j == 0:
                            rot = True
                            pom = self.find_corner("BL").get_pos()
                            rot_point = [pom[0] - 0.025, pom[1] - 0.025]
                        elif corner == "TL" and i == 0 and j == 0:
                            rot = True
                            pom = self.find_corner("BR").get_pos()
                            rot_point = [pom[0] + 0.025, pom[1] - 0.025]
                        else:
                            rot = False
                        it = PlaceItem(
                            self.scn,
                            "Object",
                            ps.pose.position.x,
                            ps.pose.position.y,
                            ps.pose.position.z,
                            conversions.q2a(ps.pose.orientation),
                            self.object_type,
                            None,
                            place_pose_changed=None,
                            fixed=False,
                            txt=False,
                            rot=rot,
                            rot_point=rot_point,
                            rotation_changed=self.items_rotation_changed,
                            parent=self,
                            horizontal=self.horizontal
                        )
                        it.setRotation(rotation)
                        it.update_point()
                        self.items.append(it)

                        if corner == "BR" or corner == "TR":
                            ps.pose.position.x += self.object_side_length_x  # BR TR
                        else:
                            ps.pose.position.x -= self.object_side_length_x  # TL BL
                    if corner == "BR" or corner == "TR":
                        ps.pose.position.x = self.pom_min[0] + self.space / 2 + self.object_side_length_x / 2  # BR a TR
                    else:
                        ps.pose.position.x = self.pom_max[0] - self.space / 2 - self.object_side_length_x / 2  # TL BL

                    if corner == "BR" or corner == "BL":
                        ps.pose.position.y -= self.object_side_length_y + self.space / 2   # BR BL
                    else:
                        ps.pose.position.y += self.object_side_length_y + self.space / 2  # TL TR
                self.previous_width = width_count
                self.previous_height = height_count

            self.last_corner = corner

            # rotation of all objects in a grid (list of all objects is handed over to the first object. When the first object is rotated, all objects are rotated)
            if self.items:
                self.items[0].set_other_items(self.items[1:])

        if finished and self.square_changed is not None:

            # even distribution of objects in the grid
            if self.items:
                new_object_length_x = ((self.pom_max[0] - self.pom_min[0]) - self.space) / self.previous_width
                new_object_length_y = ((self.pom_max[1] - self.pom_min[1]) - self.space) / self.previous_height

                for i, it in enumerate(self.items):
                    if self.last_corner == "BR" or self.last_corner == "TR":
                        new_x = self.pom_min[0] + self.space / 2 + new_object_length_x / 2 + new_object_length_x * (i % self.previous_width)
                    else:
                        new_x = self.pom_max[0] - self.space / 2 - new_object_length_x / 2 - new_object_length_x * (i % self.previous_width)

                    if self.last_corner == "BR" or self.last_corner == "BL":
                        new_y = self.pom_max[1] - self.space / 2 - new_object_length_y / 2 - new_object_length_y * (
                            i / self.previous_width)
                    else:
                        new_y = self.pom_min[1] + self.space / 2 + new_object_length_y / 2 + new_object_length_y * (
                            i / self.previous_width)
                    it.set_pos(new_x, new_y)
                    it.update_point()

                for it in self.items:   # to check if there are still some collisions
                    it.item_moved()

            self.plus_btn.setEnabled(True)
            self.minus_btn.setEnabled(True)

            if self.last_corner == "BR" or self.last_corner == "BL":  # TODO: skontrolovat ci nema byt TR a TL!!!!!!!!!!!!!!!!!!!!
                self.items.reverse()    # we want robot always to place object from furthest line

            in_collision = False
            for it in self.items:   # to check collisions
                if it.in_collision:
                    in_collision = True
                    break
            if in_collision:
                self.square_changed(self.get_square_points(), [])  # to save only grid points into the ProgramItem message
            else:
                self.square_changed(self.get_square_points(), self.items)  # to save grid points and objects into the ProgramItem message

        self.update()

    def dialog_cb(self, idx):
        if idx == 0:
            self.horizontal = False
            self.point_changed(True, "BR")
        else:
            self.horizontal = True
            self.point_changed(True, "BR")

    '''
        Method returns grid points.
    '''

    def get_square_points(self):

        pts = []

        for pt in self.pts:

            pts.append(pt.get_pos())

        return pts

    '''
        Method which is called after releasing the grid.
        It saves new positions of grid points and objects into the ProgramItem message.
        It checks collisions between grid objects and scene objects.
    '''

    def cursor_release(self):

        if self.fixed:
            return
        for it in self.items:
            it.item_moved()

        if self.square_changed is not None:
            in_collision = False
            for it in self.items:  # to check collisions
                if it.in_collision:
                    in_collision = True
                    break
            if in_collision:
                self.square_changed(self.get_square_points(), [])  # to save only grid points into the ProgramItem message
            else:
                self.square_changed(self.get_square_points(), self.items)  # to save grid points and objects into the ProgramItem message

    def cursor_press(self):

        pass

    '''
        Method saves new rotations after rotating objects.
    '''

    def items_rotation_changed(self, items):

        self.square_changed(self.get_square_points(), items)  # to save grid points and objects into the ProgramItem message

    '''
        Method returns the shape of this grid as a QPainterPath in local coordinates.
    '''

    def shape(self):

        path = QtGui.QPainterPath()
        path.addPolygon(QtGui.QPolygonF(self.square))
        return path

    '''
        Method defines the outer bounds of the item as a rectangle.
    '''

    def boundingRect(self):

        return QtCore.QRectF(self.m2pix(self.min[0]) - 2.5, self.m2pix(self.min[1]) - 2.5, self.m2pix(self.max[0] - self.min[0]) + 5, self.m2pix(self.max[1] - self.min[1]) + 5)

    '''
        Method paints the grid.
    '''

    def paint(self, painter, option, widget):

        painter.setClipRect(option.exposedRect)
        painter.setRenderHint(QtGui.QPainter.Antialiasing)

        pen = QtGui.QPen()
        pen.setStyle(QtCore.Qt.DotLine)
        pen.setWidth(5)
        pen.setBrush(QtCore.Qt.white)
        pen.setCapStyle(QtCore.Qt.RoundCap)
        pen.setJoinStyle(QtCore.Qt.RoundJoin)

        painter.setPen(pen)

        self.square = QtGui.QPolygon()

        for i in range(0, len(self.pts)):

            self.square.append(self.pts[i].pos().toPoint())

        painter.drawPolygon(self.square)
