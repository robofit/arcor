#!/usr/bin/env python

from PyQt4 import QtGui, QtCore
from item import Item
from desc_item import DescItem


class SquarePointItem(Item):

    def __init__(self,  scene,  rpm, x,  y,  parent, corner, changed=False):

        self.outline_diameter = 0.025

        super(SquarePointItem, self).__init__(scene, rpm, x, y, parent)
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

    def __init__(self,  scene,  rpm, caption, min_x, min_y, square_width, square_height, square_changed=None):

        self.caption = caption
        self.square_changed = square_changed

        super(SquareItem, self).__init__(scene, rpm, 0, 0)

        self.min = [min_x, min_y]
        self.max = [min_x + square_width, min_y + square_height]

        self.desc = DescItem(scene, rpm, self.min[0] - 0.02, self.max[1] + 0.015, self)
        self.update_text()


        self.pts = []
        self.pts.append(SquarePointItem(scene, rpm, self.min[0], self.min[1], self, "LH"))  # top-left corner
        self.pts.append(SquarePointItem(scene, rpm, self.max[0], self.min[1], self, "PH"))  # top-right corner
        self.pts.append(SquarePointItem(scene, rpm, self.max[0], self.max[1], self, "PD"))  # bottom-right corner
        self.pts.append(SquarePointItem(scene, rpm, self.min[0], self.max[1], self, "LD"))  # bottom-left corner

        self.update()

    def update_text(self):

        desc = []
        desc.append(self.caption)
        self.desc.set_content(desc)

    def find_corner(self, corner):
        for pt in self.pts:
            if pt.get_corner() == corner:
                return pt
        return None

    def point_changed(self,  finished = False):

        self.prepareGeometryChange()

        # update of bounding rect
        self.min[0] = self.pix2m(self.pts[0].x())
        self.min[1] = self.pix2m(self.pts[0].y())
        self.max[0] = self.pix2m(self.pts[0].x())
        self.max[1] = self.pix2m(self.pts[0].y())

        for pt in self.pts:

            p = (self.pix2m(pt.x()),  self.pix2m(pt.y()))

            if p[0] < self.min[0]: self.min[0] = p[0]
            if p[1] < self.min[1]: self.min[1] = p[1]

            if p[0] > self.max[0]: self.max[0] = p[0]
            if p[1] > self.max[1]: self.max[1] = p[1]

        for pt in self.pts:
            if (pt.get_corner() == "PD") and pt.get_changed():
                self.find_corner("PH").setPos(pt.x(), self.find_corner("PH").y())
                self.find_corner("LD").setPos(self.find_corner("LD").x(), pt.y())
                self.desc.setPos(self.m2pix(self.min[0] - 0.02), self.m2pix(self.max[1] + 0.015))
                pt.set_changed(False)
            elif (pt.get_corner() == "LD") and pt.get_changed():
                self.find_corner("LH").setPos(pt.x(), self.find_corner("LH").y())
                self.find_corner("PD").setPos(self.find_corner("PD").x(), pt.y())
                self.desc.setPos(self.m2pix(self.min[0] - 0.02), self.m2pix(self.max[1] + 0.015))
                pt.set_changed(False)
            elif (pt.get_corner() == "LH") and pt.get_changed():
                self.find_corner("LD").setPos(pt.x(), self.find_corner("LD").y())
                self.find_corner("PH").setPos(self.find_corner("PH").x(), pt.y())
                self.desc.setPos(self.m2pix(self.min[0] - 0.02), self.m2pix(self.max[1] + 0.015))
                pt.set_changed(False)
            elif (pt.get_corner() == "PH") and pt.get_changed():
                self.find_corner("PD").setPos(pt.x(), self.find_corner("PD").y())
                self.find_corner("LH").setPos(self.find_corner("LH").x(), pt.y())
                self.desc.setPos(self.m2pix(self.min[0] - 0.02), self.m2pix(self.max[1] + 0.015))
                pt.set_changed(False)

        self.update()

        if finished and self.square_changed is not None:
                self.square_changed()

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