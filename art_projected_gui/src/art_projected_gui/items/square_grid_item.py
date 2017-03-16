#!/usr/bin/env python

from PyQt4 import QtGui, QtCore
from item import Item
from art_projected_gui.items import PlaceItem
from desc_item import DescItem
from math import modf
from geometry_msgs.msg import PoseStamped
from helpers import conversions

translate = QtCore.QCoreApplication.translate

class SquarePointItem(Item):

    def __init__(self,  scene, x, y,  parent, corner, changed=False):

        self.outline_diameter = 0.025

        super(SquarePointItem, self).__init__(scene, x, y, parent)

        self.corner = corner
        self.changed = changed

        self.fixed = False

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

        if self.hover:
            # TODO coordinates
            painter.setBrush(QtCore.Qt.gray)
            painter.setPen(QtCore.Qt.gray)
            painter.drawEllipse(QtCore.QPoint(0,  0), es/2*1.8, es/2*1.8)

        painter.setBrush(QtCore.Qt.cyan)
        painter.setPen(QtCore.Qt.cyan)

        painter.drawEllipse(QtCore.QPoint(0,  0), es/2, es/2)



class SquareItem(Item):

    def __init__(self,  scene, caption, min_x, min_y, square_width, square_height, object_type, scene_items, square_changed=None):

        self.scn = scene
        self.caption = caption
        self.object_type = object_type
        self.scene_items = scene_items
        self.square_changed = square_changed
        space = 0.005   # pripocitava sa k bbox objektu, cize v skutocnosti je medzera medzi objektami 0.005, ale mezdi objektom a krabicou len 0.0025
        self.object_side_length = self.object_type.bbox.dimensions[0] + space

        self.previous_width = 0
        self.previous_height = 0

        self.items = []

        self.min = [min_x, min_y]
        self.max = [min_x + square_width, min_y + square_height]
        self.pom_min = [0.675, 0.32]    # pre uchovanie povodnych hodnot, pretoze v update_bound sa y znehodnoti
        self.pom_max = [0.775, 0.395]   # a potrebujem povodne hodnoty k umiestnovaniu objektov v gride

        self.orig_x = []
        self.orig_y = []

        self.desc = None

        super(SquareItem, self).__init__(scene, 0, 0)

        self.desc = DescItem(scene, self.min[0] - 0.01, self.min[1] - 0.015, self)
        self.desc.setFlag(QtGui.QGraphicsItem.ItemIgnoresTransformations)
        self.update_text()


        self.pts = []
        self.pts.append(SquarePointItem(scene, self.min[0], self.min[1], self, "BL"))  # bottom-left corner
        self.pts.append(SquarePointItem(scene, self.max[0], self.min[1], self, "BR"))  # bottom-right corner
        self.pts.append(SquarePointItem(scene, self.max[0], self.max[1], self, "TR"))  # top-right corner
        self.pts.append(SquarePointItem(scene, self.min[0], self.max[1], self, "TL"))  # top-left corner

        if self.square_changed is not None:
            self.square_changed(self.get_square_points(), self.items)   # ulozenie bodov do ProgramItem zpravy

        self.update_bound()

        self.update()

    def update_text(self):

        if self.desc is None:
            return

        desc = self.caption
        self.desc.set_content(desc)

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

    def point_changed(self,  finished=False):

        self.prepareGeometryChange()

        corner = ""

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

        if corner != "":

            width_count = int(modf(round(((self.max[0] - self.min[0]) / self.object_side_length), 5))[1])
            height_count = int(modf(round(((self.max[1] - self.min[1]) / self.object_side_length), 5))[1])
            ps = PoseStamped()

            if corner == "BR" or corner == "TR":
                ps.pose.position.x = self.pom_min[0] + self.object_side_length/2
            else:
                ps.pose.position.x = self.pom_max[0] - self.object_side_length/2

            if corner == "BR" or corner == "BL":
                ps.pose.position.y = self.pom_max[1] - self.object_side_length/2
            else:
                ps.pose.position.y = self.pom_min[1] + self.object_side_length/2
            ps.pose.orientation.w = 1.0

            if self.previous_width != width_count or self.previous_height != height_count:
                for it in self.items:
                    self.scn.removeItem(it)
                del self.items[:]
                for i in range(0, height_count):
                    for j in range(0, width_count):
                        if corner == "BR" and i == 0 and j == 0:
                            rot = True
                        elif corner == "BL" and i == 0 and j == width_count - 1:
                            rot = True
                        elif corner == "TR" and i == height_count - 1 and j == 0:
                            rot = True
                        elif corner == "TL" and i == height_count - 1 and j == width_count - 1:
                            rot = True
                        else:
                            rot = False
                        it = PlaceItem(
                            self.scn,
                            "Object",
                            ps.pose.position.x,
                            ps.pose.position.y,
                            self.object_type,
                            None,
                            place_pose_changed=None,
                            fixed=False,
                            yaw=conversions.quaternion2yaw(ps.pose.orientation),
                            txt=False,
                            rot=rot
                        )
                        self.items.append(it)

                        if corner == "BR" or corner == "TR":
                            ps.pose.position.x += self.object_side_length  # BR TR
                        else:
                            ps.pose.position.x -= self.object_side_length  # TL BL
                    if corner == "BR" or corner == "TR":
                        ps.pose.position.x = self.pom_min[0] + self.object_side_length/2    # BR a TR
                    else:
                        ps.pose.position.x = self.pom_max[0] - self.object_side_length/2  # TL BL

                    if corner == "BR" or corner == "BL":
                        ps.pose.position.y -= self.object_side_length    # BR BL
                    else:
                        ps.pose.position.y += self.object_side_length  # TL TR
                self.previous_width = width_count
                self.previous_height = height_count

            if self.items:
                self.items[0].set_other_items(self.items)


        if finished and self.square_changed is not None:
            self.square_changed(self.get_square_points(), self.items)   # ulozenie bodov do ProgramItem zpravy

        self.update()

    def get_square_points(self):

        pts = []

        for pt in self.pts:

            pts.append(pt.get_pos())

        return pts

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

        square = QtGui.QPolygon()

        for i in range(0,  len(self.pts)):
            
            square.append(self.pts[i].pos().toPoint())

        painter.drawPolygon(square)