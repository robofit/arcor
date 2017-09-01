#!/usr/bin/env python

import unittest
import rostest
import sys
from art_projected_gui.gui import UICore
from art_projected_gui.items import ObjectItem, PlaceItem
from PyQt4.QtGui import QApplication
from art_msgs.msg import ObjectType
import rospy
from shape_msgs.msg import SolidPrimitive
from geometry_msgs.msg import PoseStamped

app = QApplication(sys.argv)


class TestUICore(unittest.TestCase):

    def setUp(self):

        self.ui_core = UICore(0, 0, 2, 1, 1000, 1234)

        self.type1 = ObjectType()
        self.type1.name = "type1"
        self.type1.bbox.type = SolidPrimitive.BOX
        self.type1.bbox.dimensions = [0.1, 0.1, 0.1]

        self.type2 = ObjectType()
        self.type2.name = "type2"
        self.type2.bbox.type = SolidPrimitive.BOX
        self.type2.bbox.dimensions = [0.1, 0.1, 0.1]

        self.ps = PoseStamped()
        self.ps.pose.orientation.w = 1.0

    def test_select_object_type(self):

        self.ui_core.add_object("id1", self.type1, 0.5, 0.5, 0.0, [0, 0, 0, 1])
        self.ui_core.add_object("id2", self.type2, 0.5, 0.5, 0.0, [0, 0, 0, 1])
        self.ui_core.add_object("id3", self.type2, 0.5, 0.5, 0.0, [0, 0, 0, 1])

        self.ui_core.select_object_type("type1")

        self.assertEquals(self.ui_core.get_object("id1").selected, True, "test_select_object_type")
        self.assertEquals(self.ui_core.get_object("id2").selected, False, "test_select_object_type")
        self.assertEquals(self.ui_core.get_object("id3").selected, False, "test_select_object_type")

        self.ui_core.select_object_type("type2")

        self.assertEquals(self.ui_core.get_object("id1").selected, False, "test_select_object_type")
        self.assertEquals(self.ui_core.get_object("id2").selected, True, "test_select_object_type")
        self.assertEquals(self.ui_core.get_object("id3").selected, True, "test_select_object_type")

        self.ui_core.select_object_type("type1", unselect_others=False)

        self.assertEquals(self.ui_core.get_object("id1").selected, True, "test_select_object_type")
        self.assertEquals(self.ui_core.get_object("id2").selected, True, "test_select_object_type")
        self.assertEquals(self.ui_core.get_object("id3").selected, True, "test_select_object_type")

    def test_get_object(self):

        self.ui_core.add_object("id1", self.type1, 0.5, 0.5, 0.0, [0, 0, 0, 1])
        self.ui_core.add_object("id2", self.type1, 0.5, 0.5, 0.0, [0, 0, 0, 1])

        self.assertIsNotNone(self.ui_core.get_object("id1"), "test_get_object")
        self.assertEquals(self.ui_core.get_object("id1").object_id, "id1", "test_get_object")

        self.assertIsNone(self.ui_core.get_object("non_existent_id"), "test_get_object")

    def test_remove_object(self):

        self.ui_core.add_object("id1", self.type1, 0.5, 0.5, 0.0, [0, 0, 0, 1])
        self.ui_core.add_object("id2", self.type1, 0.5, 0.5, 0.0, [0, 0, 0, 1])

        self.assertEquals(len(list(self.ui_core.get_scene_items_by_type(ObjectItem))), 2, "test_remove_object")

        self.assertEquals(self.ui_core.remove_object("id1"), True, "test_remove_object")
        self.assertEquals(len(list(self.ui_core.get_scene_items_by_type(ObjectItem))), 1, "test_remove_object")

        self.assertEquals(self.ui_core.remove_object("id2"), True, "test_remove_object")
        self.assertEquals(len(list(self.ui_core.get_scene_items_by_type(ObjectItem))), 0, "test_remove_object")

        self.assertEquals(self.ui_core.remove_object("id1"), False, "test_remove_object")

    def test_get_by_type(self):

        self.ui_core.add_object("id1", self.type1, 0.5, 0.5, 0.0, [0, 0, 0, 1])
        self.ui_core.add_place("caption", self.ps, self.type1)

        self.assertEquals(len(list(self.ui_core.get_scene_items_by_type(ObjectItem))), 1, "test_get_by_type")
        self.assertEquals(len(list(self.ui_core.get_scene_items_by_type(PlaceItem))), 1, "test_get_by_type")

    def test_clear_places(self):

        self.ui_core.add_object("id1", self.type1, 0.5, 0.5, 0.0, [0, 0, 0, 1])
        self.ui_core.add_place("caption", self.ps, self.type1)
        self.ui_core.clear_places()

        self.assertEquals(len(list(self.ui_core.get_scene_items_by_type(ObjectItem))), 1, "test_clear_places")
        self.assertEquals(len(list(self.ui_core.get_scene_items_by_type(PlaceItem))), 0, "test_clear_places")


if __name__ == '__main__':

    rospy.init_node('test_node')
    rostest.run('art_projected_gui', 'test_ui_core', TestUICore, sys.argv)
