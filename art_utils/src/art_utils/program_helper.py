#!/usr/bin/env python

import rospy
from art_msgs.msg import Program, ProgramItem as Pi
from geometry_msgs.msg import Pose, Polygon


class ProgramHelper():

    """ProgramHelper simplifies work with Program message.

        The class can load and check Program message. It has no internal state.
        It only helps to find next block/item id after success or failure (without iterating over all blocks/items).

    """

    ITEMS_USING_OBJECT = [Pi.PICK_FROM_POLYGON, Pi.PICK_FROM_FEEDER, Pi.PICK_OBJECT_ID, Pi.PLACE_TO_POSE, Pi.PLACE_TO_GRID, Pi.DRILL_POINTS]
    ITEMS_USING_POSE = [Pi.PICK_FROM_FEEDER, Pi.PLACE_TO_POSE, Pi.DRILL_POINTS, Pi.PLACE_TO_GRID]
    ITEMS_USING_POLYGON = [Pi.PICK_FROM_POLYGON, Pi.DRILL_POINTS, Pi.PLACE_TO_GRID]
    PICK_ITEMS = [Pi.PICK_FROM_POLYGON, Pi.PICK_FROM_FEEDER, Pi.PICK_OBJECT_ID]
    PLACE_ITEMS = [Pi.PLACE_TO_POSE, Pi.PLACE_TO_GRID]

    def __init__(self):

        self._cache = {}
        self._prog = None

    def load(self, prog, template=False):

        if not isinstance(prog, Program):
            rospy.logerr("Invalid argument. Should be Program message.")
            return False

        cache = {}

        if len(prog.blocks) == 0:

            rospy.logerr("Program with zero blocks!")
            return False

        for block_idx in range(0, len(prog.blocks)):

            block = prog.blocks[block_idx]

            if block.id in cache:

                rospy.logerr("Duplicate block id: " + str(block.id))
                return False

            if block.id == 0:

                rospy.logerr("Invalid block id: " + str(block.id))
                return False

            cache[block.id] = {}

            cache[block.id]["idx"] = block_idx
            cache[block.id]["on_success"] = block.on_success
            cache[block.id]["on_failure"] = block.on_failure
            cache[block.id]["items"] = {}

            if len(block.items) == 0:

                rospy.logerr("Block with zero items!")
                return False

            for item_idx in range(0, len(block.items)):

                item = block.items[item_idx]

                if item.id in cache[block.id]["items"]:

                    rospy.logerr("Duplicate item id: " + str(item.id) + " (block id: " + str(block.id) + ")")
                    return False

                if item.id == 0:

                    rospy.logerr("Invalid item id: " + str(item.id) + " (block id: " + str(block.id) + ")")
                    return False

                cache[block.id]["items"][item.id] = {}
                cache[block.id]["items"][item.id]["idx"] = item_idx
                cache[block.id]["items"][item.id]["on_success"] = item.on_success
                cache[block.id]["items"][item.id]["on_failure"] = item.on_failure

        self._prog = prog
        self._cache = cache

        # now the cache is done, let's make some simple checks
        for k, v in cache.iteritems():

            # TODO refactor into separate method check_item
            # 0 means jump to the end
            if v["on_success"] != 0 and v["on_success"] not in cache:

                rospy.logerr("Block id: " + str(k) + " has invalid on_success: " + str(v["on_success"]))
                return False

            if v["on_failure"] != 0 and v["on_failure"] not in cache:

                rospy.logerr("Block id: " + str(k) + " has invalid on_success: " + str(v["on_success"]))
                return False

            for kk, vv in v["items"].iteritems():

                # 0 means jump to the end
                if vv["on_success"] != 0 and vv["on_success"] not in cache[k]["items"]:

                    rospy.logerr("Block id: " + str(k) + ", item id: " + str(kk) + " has invalid on_success: " + str(vv["on_success"]))
                    return False

                if vv["on_failure"] != 0 and vv["on_failure"] not in cache[k]["items"]:

                    rospy.logerr("Block id: " + str(k) + ", item id: " + str(kk) + " has invalid on_failure: " + str(vv["on_failure"]))
                    return False

                item = prog.blocks[v["idx"]].items[vv["idx"]]

                # any reference should exist in the same block
                for ref in item.ref_id:

                    if ref not in cache[k]["items"]:

                        rospy.logerr("Block id: " + str(k) + ", item id: " + str(kk) + " has invalid ref_id: " + str(ref))
                        return False

                # at least one 'object' mandatory for following types
                if item.type in self.ITEMS_USING_OBJECT:

                    res = self.get_object(k, kk)

                    if res is None:
                        rospy.logerr("No 'object' for block id: " + str(k) + ", item id: " + str(kk) + "!")
                        return False

                # at least one 'pose' mandatory for following types
                if item.type in self.ITEMS_USING_POSE:

                    res = self.get_pose(k, kk)

                    if res is None:
                        rospy.logerr("No 'pose' for block id: " + str(k) + ", item id: " + str(kk) + "!")
                        return False

                # at least one 'polygon' mandatory for following types
                if item.type in self.ITEMS_USING_POLYGON:

                    res = self.get_polygon(k, kk)

                    if res is None:
                        rospy.logerr("No 'polygon' for block id: " + str(k) + ", item id: " + str(kk) + "!")
                        return False

                # check if PLACE_* instruction has correct ref_id(s) - should be set and point to PICK_*
                if item.type in self.PLACE_ITEMS:

                    if len(item.ref_id) == 0:

                        rospy.logerr("Block id: " + str(k) + ", item id: " + str(kk) + " has NO ref_id!")
                        return False

                    for ref_id in item.ref_id:

                        ref_msg = self.get_item_msg(k, ref_id)

                        if ref_msg.type not in self.PICK_ITEMS:

                            rospy.logerr("Block id: " + str(k) + ", item id: " + str(kk) + " has ref_id which is not PICK_*!")
                            return False

                # TODO refactor into separate method
                if template:

                    for i in range(0, len(item.object)):
                        item.object[i] = ""

                    # for stamped types we want to keep header (frame_id)
                    for polygon in item.polygon:
                        polygon.polygon = Polygon()

                    for pose in item.pose:
                        pose.pose = Pose()

        return True

    def get_program(self):

        return self._prog

    def get_program_id(self):

        return self._prog.header.id

    def get_block_msg(self, block_id):

        block_idx = self._cache[block_id]["idx"]
        return self._prog.blocks[block_idx]

    def get_block_ids(self):

        return self._cache.keys()

    def get_items_ids(self, block_id):

        return self._cache[block_id]["items"].keys()

    def get_first_block_id(self):

        return min(self._cache, key=self._cache.get)

    def get_first_item_id(self, block_id=None):

        if block_id is None:
            block_id = self.get_first_block_id()
        items = self._cache[block_id]["items"]
        item_id = min(items, key=items.get)
        return (block_id, item_id)

    def get_item_msg(self, block_id, item_id):

        block_idx = self._cache[block_id]["idx"]
        item_idx = self._cache[block_id]["items"][item_id]["idx"]
        return self._prog.blocks[block_idx].items[item_idx]

    def set_item_msg(self, block_id, msg):

        block_idx = self._cache[block_id]["idx"]
        item_idx = self._cache[block_id]["items"][msg.id]["idx"]

        omsg = self._prog.blocks[block_idx].items[item_idx]

        if omsg.on_success != msg.on_success or omsg.on_failure != msg.on_failure:
            raise ValueError("Attempt to change program structure!")

        self._prog.blocks[block_idx].items[item_idx] = msg

    def _get_block_on(self, block_id, what):

        return self._cache[block_id][what]

    def _get_item_on(self, block_id, item_id, what):

        item_id_on = self._cache[block_id]["items"][item_id][what]

        # TODO make constant in msg for it
        if item_id_on == 0:

            next_block_id = self. _get_block_on(block_id, what)

            if next_block_id == 0:

                return (0, 0)  # end of program

            return self.get_first_item_id(next_block_id)

        else:

            return (block_id, item_id_on)

    def get_id_on_success(self, block_id, item_id):

        return self._get_item_on(block_id, item_id, "on_success")

    def get_id_on_failure(self, block_id, item_id):

        return self._get_item_on(block_id, item_id, "on_failure")

    def get_block_on_success(self, block_id):

        return self._get_block_on(block_id, "on_success")

    def get_block_on_failure(self, block_id):

        return self._get_block_on(block_id, "on_failure")

    def get_item_type(self, block_id, item_id):

        msg = self.get_item_msg(block_id, item_id)
        return msg.type

    def item_requires_learning(self, block_id, item_id):

        return self.get_item_type(block_id, item_id) in [Pi.PICK_FROM_POLYGON, Pi.PICK_FROM_FEEDER, Pi.PICK_OBJECT_ID, Pi.PLACE_TO_POSE, Pi.PLACE_TO_GRID, Pi.DRILL_POINTS]

    def _check_for_pose(self, msg):

        if msg.type not in self.ITEMS_USING_POSE:
            raise ValueError("Instruction type " + str(msg.type) + " does not use 'object'.")

    def _check_for_object(self, msg):

        if msg.type not in self.ITEMS_USING_OBJECT:

            raise ValueError("Instruction type " + str(msg.type) + " does not use 'object'.")

    def _check_for_polygon(self, msg):

        if msg.type not in self.ITEMS_USING_POLYGON:

            raise ValueError("Instruction type " + str(msg.type) + " does not use 'polygon'.")

    def get_pose(self, block_id, item_id):

        msg = self.get_item_msg(block_id, item_id)

        self._check_for_pose(msg)

        if len(msg.pose) > 0:

            return msg.pose, item_id

        for ref_id in msg.ref_id:

            try:
                ret = self.get_pose(block_id, ref_id)
            except ValueError:
                continue

            if ret is not None:
                return ret

        return None

    def get_object(self, block_id, item_id):

        msg = self.get_item_msg(block_id, item_id)

        self._check_for_object(msg)

        if len(msg.object) > 0:

            return msg.object, item_id

        for ref_id in msg.ref_id:

            try:
                ret = self.get_object(block_id, ref_id)
            except ValueError:
                continue

            if ret is not None:
                return ret

        return None

    def get_polygon(self, block_id, item_id):

        msg = self.get_item_msg(block_id, item_id)

        self._check_for_polygon(msg)

        if len(msg.polygon) > 0:

            return msg.polygon, item_id

        for ref_id in msg.ref_id:

            try:
                ret = self.get_polygon(block_id, ref_id)
            except ValueError:
                continue

            if ret is not None:
                return ret

        return None

    def is_pose_set(self, block_id, item_id):

        ret = self.get_pose(block_id, item_id)

        if ret is None:
            return ValueError("'pose' does not exist in item or its refs.")

        for p in ret[0]:
            if p.pose == Pose():
                return False

        return True

    def is_object_set(self, block_id, item_id):

        ret = self.get_object(block_id, item_id)

        if ret is None:
            return ValueError("'object' does not exist in item or its refs.")

        for obj in ret[0]:
            if obj == "":
                return False

        return True

    def is_polygon_set(self, block_id, item_id):

        ret = self.get_polygon(block_id, item_id)

        if ret is None:
            return ValueError("'polygon' does not exist in item or its refs.")

        for poly in ret[0]:
            if poly.polygon == Polygon():
                return False

        return True

    def program_learned(self):

        blocks = self.get_block_ids()

        for block_id in blocks:

            if not self.block_learned(block_id):
                return False

        return True

    def block_learned(self, block_id):

        items = self.get_items_ids(block_id)

        for item_id in items:

            if self.item_learned(block_id, item_id) is False:
                return False

        return True

    def item_has_nothing_to_set(self, block_id, item_id):

        msg = self.get_item_msg(block_id, item_id)

        return len(msg.object) == 0 and len(msg.pose) == 0 and len(msg.polygon) == 0

    def item_learned(self, block_id, item_id):

        if not self.item_requires_learning(block_id, item_id):
            return None

        msg = self.get_item_msg(block_id, item_id)

        arr = ((self.ITEMS_USING_POLYGON, self.is_polygon_set), (self.ITEMS_USING_POSE, self.is_pose_set), (self.ITEMS_USING_OBJECT, self.is_object_set))

        for ar in arr:

            if msg.type in ar[0]:

                if not ar[1](block_id, item_id):
                    return False

        # TODO how and where to detect non-supported types?
        # raise NotImplementedError("Not yet supported item type.")

        return True
