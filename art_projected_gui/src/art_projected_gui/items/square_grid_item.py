#!/usr/bin/env python

from PyQt4 import QtGui, QtCore
from item import Item
from art_projected_gui.items import PlaceItem, ButtonItem, ObjectItem
from desc_item import DescItem
from math import modf
from geometry_msgs.msg import PoseStamped
# from art_msgs.msg import ObjectType
# from shape_msgs.msg import SolidPrimitive
from helpers import conversions

translate = QtCore.QCoreApplication.translate

class SquarePointItem(Item):

    def __init__(self,  scene, x, y,  parent, corner, fixed, changed=False):

        self.outline_diameter = 0.025

        super(SquarePointItem, self).__init__(scene, x, y, parent)

        self.corner = corner
        self.changed = changed

        self.fixed = fixed

    def get_corner(self):

        return self.corner

    def set_changed(self, changed):

        self.changed = changed

    def get_changed(self):

        return self.changed

    def boundingRect(self):

        es = self.m2pix(self.outline_diameter*1.8)

        return QtCore.QRectF(-es/2, -es/2, es, es)

    def shape(self):

        path = QtGui.QPainterPath()
        es = self.m2pix(self.outline_diameter)
        path.addEllipse(QtCore.QPoint(0,  0),  es/2,  es/2)
        return path

    def item_moved(self):

        self.changed = True
        self.parentItem().point_changed()

    def cursor_release(self):

        self.parentItem().point_changed(True)

    def paint(self, painter, option, widget):

        painter.setClipRect(option.exposedRect)
        painter.setRenderHint(QtGui.QPainter.Antialiasing)

        es = self.m2pix(self.outline_diameter)

        if self.hover and not self.fixed:
            # TODO coordinates
            painter.setBrush(QtCore.Qt.gray)
            painter.setPen(QtCore.Qt.gray)
            painter.drawEllipse(QtCore.QPoint(0,  0), es/2*1.8, es/2*1.8)

        painter.setBrush(QtCore.Qt.cyan)
        painter.setPen(QtCore.Qt.cyan)

        painter.drawEllipse(QtCore.QPoint(0,  0), es/2, es/2)



class SquareItem(Item):

    def __init__(self,  scene, caption, min_x, min_y, square_width, square_height, object_type, poses, grid_points, scene_items, square_changed=None, fixed=False):

        self.scn = scene
        self.caption = caption
        self.object_type = object_type

        # self.horizontal_item = ObjectType()
        # self.horizontal_item.name = "horizontal"
        # self.horizontal_item.bbox.type = SolidPrimitive.BOX
        # self.horizontal_item.bbox.dimensions.append(self.object_type.bbox.dimensions[2])
        # self.horizontal_item.bbox.dimensions.append(self.object_type.bbox.dimensions[1])
        # self.horizontal_item.bbox.dimensions.append(self.object_type.bbox.dimensions[0])

        self.scene_items = scene_items
        self.square_changed = square_changed
        self.space = 0.005   # pripocitava sa k bbox objektu, cize v skutocnosti je medzera medzi objektami 'space', ale mezdi objektom a krabicou len 'space'/2
        # self.object_side_length_x = self.horizontal_item.bbox.dimensions[0] + self.space
        # self.object_side_length_y = self.horizontal_item.bbox.dimensions[1] + self.space
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
            self.pom_min = self.min     # pre uchovanie povodnych hodnot, pretoze v update_bound sa y znehodnoti
            self.pom_max = self.max     # a potrebujem povodne hodnoty k umiestnovaniu objektov v gride
        else:
            self.min = list(min(grid_points))
            self.max = list(max(grid_points))

        self.orig_x = []
        self.orig_y = []

        self.desc = None

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
            for pose in poses:
                it = PlaceItem(
                    self.scn,
                    "Object",
                    pose.pose.position.x,
                    pose.pose.position.y,
                    # self.horizontal_item,
                    self.object_type,
                    None,
                    place_pose_changed=None,
                    fixed=True,
                    yaw=conversions.quaternion2yaw(pose.pose.orientation),
                    txt=False
                )
                self.items.append(it)
        else:
            for i, pose in enumerate(poses):
                rot_point = None  # sluzi na ulozenie xy suradnic pre umiestnenie bodu na rotovanie objektov v gride
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
                    # self.horizontal_item,
                    self.object_type,
                    None,
                    place_pose_changed=None,
                    fixed=False,
                    yaw=conversions.quaternion2yaw(pose.pose.orientation),
                    txt=False,
                    rot=rot,
                    rot_point=rot_point,
                    rotation_changed=self.items_rotation_changed
                )
                it.update_point()
                self.items.append(it)

            if self.items:
                self.items[0].set_other_items(self.items[1:])   # riesi rotaciu objektov (preda sa pole objektov prvemu objektu, s ktorym ked sa rotuje, rotuje aj s ostatnymi)
                self.plus_btn.setEnabled(True)  # mozno zvacsovat space
                self.minus_btn.setEnabled(True)  # mozno zmensovat space


        if self.square_changed is not None:
            self.square_changed(self.get_square_points(), self.items)   # ulozenie bodov do ProgramItem zpravy

        self.update_bound()
        self.update()

    def update_text(self):

        if self.desc is None:
            return

        desc = self.caption
        self.desc.set_content(desc)

    def plus_clicked(self, btn):
        self.space += 0.01
        self.point_changed(True, self.last_corner)

    def minus_clicked(self, btn):
        if self.space >= 0.01:
            self.space -= 0.01
            self.point_changed(True, self.last_corner)

    def update_bound(self):

        self.min[0] = self.pix2m(self.pts[0].x())
        self.min[1] = self.pix2m(self.pts[0].y())
        self.max[0] = self.pix2m(self.pts[0].x())
        self.max[1] = self.pix2m(self.pts[0].y())

        self.orig_x = []
        self.orig_y = []
        for pt in self.pts:

            p = (self.pix2m(pt.x()), self.pix2m(pt.y()))

            if p[0] < self.min[0]: self.min[0] = p[0]
            if p[1] < self.min[1]: self.min[1] = p[1]

            if p[0] > self.max[0]: self.max[0] = p[0]
            if p[1] > self.max[1]: self.max[1] = p[1]

            point = pt.get_pos()
            self.orig_x.append(point[0])
            self.orig_y.append(point[1])

        self.pom_min = [min(self.orig_x), min(self.orig_y)]
        self.pom_max = [max(self.orig_x), max(self.orig_y)]


    def find_corner(self, corner):
        for pt in self.pts:
            if pt.get_corner() == corner:
                return pt
        return None

    def point_changed(self,  finished=False, corner=""):

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

            # self.object_side_length_x = self.horizontal_item.bbox.dimensions[0] + self.space
            # self.object_side_length_y = self.horizontal_item.bbox.dimensions[1] + self.space
            self.object_side_length_x = self.object_type.bbox.dimensions[0] + self.space
            self.object_side_length_y = self.object_type.bbox.dimensions[1] + self.space

            width_count = int(modf(round(((self.max[0] - self.min[0]) / self.object_side_length_x), 5))[1])
            height_count = int(modf(round(((self.max[1] - self.min[1]) / self.object_side_length_y), 5))[1])
            if self.previous_width != width_count or self.previous_height != height_count:
                ps = PoseStamped()
                if corner == "BR" or corner == "TR":
                    ps.pose.position.x = self.pom_min[0] + self.object_side_length_x/2
                else:
                    ps.pose.position.x = self.pom_max[0] - self.object_side_length_x/2

                if corner == "BR" or corner == "BL":
                    ps.pose.position.y = self.pom_max[1] - self.object_side_length_y/2
                else:
                    ps.pose.position.y = self.pom_min[1] + self.object_side_length_y/2
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
                        rot_point = None    # sluzi na ulozenie xy suradnic pre umiestnenie bodu na rotovanie objektov v gride
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
                            # self.horizontal_item,
                            self.object_type,
                            None,
                            place_pose_changed=None,
                            fixed=False,
                            yaw=conversions.quaternion2yaw(ps.pose.orientation),
                            txt=False,
                            rot=rot,
                            rot_point=rot_point,
                            rotation_changed=self.items_rotation_changed
                        )
                        it.setRotation(rotation)
                        it.update_point()
                        self.items.append(it)

                        if corner == "BR" or corner == "TR":
                            ps.pose.position.x += self.object_side_length_x  # BR TR
                        else:
                            ps.pose.position.x -= self.object_side_length_x  # TL BL
                    if corner == "BR" or corner == "TR":
                        ps.pose.position.x = self.pom_min[0] + self.object_side_length_x/2    # BR a TR
                    else:
                        ps.pose.position.x = self.pom_max[0] - self.object_side_length_x/2  # TL BL

                    if corner == "BR" or corner == "BL":
                        ps.pose.position.y -= self.object_side_length_y    # BR BL
                    else:
                        ps.pose.position.y += self.object_side_length_y  # TL TR
                self.previous_width = width_count
                self.previous_height = height_count

                # for it in self.items:   # kontrola ci nie je kolizia (az po vykresleni vsetkych, lebo inak by prvy bol biely)
                #     it.item_moved()

            self.last_corner = corner

            # riesi rotaciu objektov (preda sa pole objektov prvemu objektu, s ktorym ked sa rotuje, rotuje aj s ostatnymi)
            if self.items:
                self.items[0].set_other_items(self.items[1:])

        if finished and self.square_changed is not None:

            # upravuje rozmiestnenie objektov v gride, aby boli cca rovnako daleko od stran gridu
            if self.items:
                new_object_length_x = (self.pom_max[0] - self.pom_min[0]) / self.previous_width
                new_object_length_y = (self.pom_max[1] - self.pom_min[1]) / self.previous_height

                for i, it in enumerate(self.items):
                    if self.last_corner == "BR" or self.last_corner == "TR":
                        new_x = self.pom_min[0] + new_object_length_x / 2 + new_object_length_x * (i % self.previous_width)
                    else:
                        new_x = self.pom_max[0] - new_object_length_x / 2 - new_object_length_x * (i % self.previous_width)

                    if self.last_corner == "BR" or self.last_corner == "BL":
                        new_y = self.pom_max[1] - new_object_length_y / 2 - new_object_length_y * (
                        i / self.previous_width)
                    else:
                        new_y = self.pom_min[1] + new_object_length_y / 2 + new_object_length_y * (
                        i / self.previous_width)
                    it.set_pos(new_x, new_y)
                    it.update_point()   # nutne updatnut do povodnej polohy, lebo ked sa pohne s objektom, tak sa pohne zaroven s bodom na rotovanie

                for it in self.items:   # nutne, aby sa pozrelo ci su este stale v kolizii
                    # if it.collidesWithPath(self.shape()):
                    #     print "Ano"
                    it.item_moved()

            self.plus_btn.setEnabled(True)  # uz mozno zvacsovat space
            self.minus_btn.setEnabled(True)  # uz mozno zmensovat space

            if self.last_corner == "BR" or self.last_corner == "BL":
                self.items.reverse()    # aby robot ukladal od najvzdialenejsieho radu, aby mu ulozene objekty nezavadzali

            in_collision = False
            for it in self.items:   # kontrola ci nie su objekty v kolizii
                if it.in_collision:
                    in_collision = True
                    break
            if in_collision:
                self.square_changed(self.get_square_points(), [])  # ulozenie bodov do ProgramItem zpravy
            else:
                self.square_changed(self.get_square_points(), self.items)  # ulozenie bodov do ProgramItem zpravy


        self.update()

    def get_square_points(self):

        pts = []

        for pt in self.pts:

            pts.append(pt.get_pos())

        return pts

    def cursor_release(self):

        if self.fixed:
            return
        for it in self.items:   # kontrola ci po presune celeho gridu nie su objekty v kolizii s niecim v scene
            it.item_moved()

        if self.square_changed is not None:
            in_collision = False
            for it in self.items:  # kontrola ci nie su objekty v kolizii
                if it.in_collision:
                    in_collision = True
                    break
            if in_collision:
                self.square_changed(self.get_square_points(), [])  # ulozenie bodov do ProgramItem zpravy
            else:
                self.square_changed(self.get_square_points(), self.items)  # ulozenie bodov do ProgramItem zpravy

    def cursor_press(self):

        pass

    def items_rotation_changed(self, items):

        self.square_changed(self.get_square_points(), items)  # ulozenie bodov do ProgramItem zpravy

    def shape(self):

        path = QtGui.QPainterPath()
        path.addPolygon(QtGui.QPolygonF(self.square))
        return path

    def boundingRect(self):

        return QtCore.QRectF(self.m2pix(self.min[0])-2.5,  self.m2pix(self.min[1])-2.5, self.m2pix(self.max[0]-self.min[0])+5, self.m2pix(self.max[1]-self.min[1])+5)

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

        for i in range(0,  len(self.pts)):
            
            self.square.append(self.pts[i].pos().toPoint())

        painter.drawPolygon(self.square)