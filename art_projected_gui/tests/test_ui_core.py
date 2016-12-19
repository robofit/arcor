#!/usr/bin/env python

import unittest
from gui.ui_core import UICore
from items import ObjectItem
import os.path
import sys
from PyQt4.QtGui import QApplication
from PyQt4.QtTest import QTest

sys.path = [os.path.abspath(os.path.dirname(__file__))] + sys.path
app = QApplication(sys.argv)


class TestUICore(unittest.TestCase):

    def setUp(self):

        self.ui_core = UICore(0,  0,  2, 1, 1000)

    def test_select_object_type(self):

        self.ui_core.add_object("id1",  "type1",  0.5,  0.5)
        self.ui_core.add_object("id2",  "type2",  0.5,  0.5)
        self.ui_core.add_object("id3",  "type2",  0.5,  0.5)

        self.ui_core.select_object_type("type1")

        self.assertEquals(self.ui_core.get_object("id1").selected, True, "test_select_object_type")
        self.assertEquals(self.ui_core.get_object("id2").selected, False, "test_select_object_type")
        self.assertEquals(self.ui_core.get_object("id3").selected, False, "test_select_object_type")

        self.ui_core.select_object_type("type2")

        self.assertEquals(self.ui_core.get_object("id1").selected, False, "test_select_object_type")
        self.assertEquals(self.ui_core.get_object("id2").selected, True, "test_select_object_type")
        self.assertEquals(self.ui_core.get_object("id3").selected, True, "test_select_object_type")

        self.ui_core.select_object_type("type1",  unselect_others=False)

        self.assertEquals(self.ui_core.get_object("id1").selected, True, "test_select_object_type")
        self.assertEquals(self.ui_core.get_object("id2").selected, True, "test_select_object_type")
        self.assertEquals(self.ui_core.get_object("id3").selected, True, "test_select_object_type")

    def test_get_object(self):

        self.ui_core.add_object("id1",  "type1",  0.5,  0.5)
        self.ui_core.add_object("id2",  "type1",  0.5,  0.5)

        self.assertIsNotNone(self.ui_core.get_object("id1"), "test_get_object")
        self.assertEquals(self.ui_core.get_object("id1").object_id,  "id1", "test_get_object")

        self.assertIsNone(self.ui_core.get_object("non_existent_id"), "test_get_object")

    def test_remove_object(self):

        self.ui_core.add_object("id1",  "type1",  0.5,  0.5)
        self.ui_core.add_object("id2",  "type1",  0.5,  0.5)

        self.assertEquals(len(list(self.ui_core.get_scene_items_by_type(ObjectItem))),  2, "test_remove_object")

        self.assertEquals(self.ui_core.remove_object("id1"),  True,  "test_remove_object")
        self.assertEquals(len(list(self.ui_core.get_scene_items_by_type(ObjectItem))),  1, "test_remove_object")

        self.assertEquals(self.ui_core.remove_object("id2"),  True,  "test_remove_object")
        self.assertEquals(len(list(self.ui_core.get_scene_items_by_type(ObjectItem))),  0, "test_remove_object")

        self.assertEquals(self.ui_core.remove_object("id1"),  False,  "test_remove_object")


if __name__ == '__main__':
    import rosunit
    rosunit.unitrun("art_projected_gui", 'test_ui_core', TestUICore)
