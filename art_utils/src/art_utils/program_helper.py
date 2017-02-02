#!/usr/bin/env python

import rospy
from art_msgs.msg import Program,  ProgramItem
from geometry_msgs.msg import PoseStamped, PolygonStamped


class ProgramHelper():

    """ProgramHelper simplifies work with Program message.

        The class can load and check Program message. It has no internal state.
        It only helps to find next block/item id after success or failure (without iterating over all blocks/items).

    """

    def __init__(self):

        self.cache = {}
        self.prog = None

    def load(self, prog,  template=False):

        if not isinstance(prog,  Program):
            rospy.logerr("Invalid argument. Should be Program message.")
            return False

        cache = {}

        if len(prog.blocks) == 0:

            rospy.logerr("Program with zero blocks!")
            return False

        for block_idx in range(0,  len(prog.blocks)):

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

            for item_idx in range(0,  len(block.items)):

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

        # now the cache is done, let's make some simple checks
        for k,  v in cache.iteritems():

            # 0 means jump to the end
            if v["on_success"] != 0 and v["on_success"] not in cache:

                rospy.logerr("Block id: " + str(k) + " has invalid on_success: " + str(v["on_success"]))
                return False

            if v["on_failure"] != 0 and v["on_failure"] not in cache:

                rospy.logerr("Block id: " + str(k) + " has invalid on_success: " + str(v["on_success"]))
                return False

            for kk,  vv in v["items"].iteritems():

                # 0 means jump to the end
                if vv["on_success"] != 0 and vv["on_success"] not in cache[k]["items"]:

                    rospy.logerr("Block id: " + str(k) + ", item id: " + str(kk) + " has invalid on_success: " + str(vv["on_success"]))
                    return False

                if vv["on_failure"] != 0 and vv["on_failure"] not in cache[k]["items"]:

                    rospy.logerr("Block id: " + str(k) + ", item id: " + str(kk) + " has invalid on_failure: " + str(vv["on_failure"]))
                    return False

                item = prog.blocks[v["idx"]].items[vv["idx"]]

                if template:

                    item.object = ""
                    item.pick_pose = PoseStamped()
                    item.pick_polygon = PolygonStamped()
                    item.place_pose = PoseStamped()
                    item.place_polygon = PolygonStamped()

        self.prog = prog
        self.cache = cache
        return True

    def get_program_id(self):

        if self.prog is None:
            return None

        return self.prog.header.id

    def get_block_msg(self,  block_id):

        if block_id not in self.cache:
            return None

        block_idx = self.cache[block_id]["idx"]
        return self.prog.blocks[block_idx]

    def get_block_ids(self):

        return self.cache.keys()

    def get_items_ids(self, block_id):

        return self.cache[block_id]["items"].keys()

    def get_first_block_id(self):

        if len(self.cache) == 0:
            return None

        return min(self.cache,  key=self.cache.get)

    def get_first_item_id(self):

        if len(self.cache) == 0:
            return None

        block_id = self.get_first_block_id()
        items = self.cache[block_id]["items"]
        item_id = min(items,  key=items.get)
        return (block_id,  item_id)

    def get_item_msg(self,  block_id,  item_id):

        if block_id not in self.cache or item_id not in self.cache[block_id]["items"]:
            return None

        block_idx = self.cache[block_id]["idx"]
        item_idx = self.cache[block_id]["items"][item_id]["idx"]
        return self.prog.blocks[block_idx].items[item_idx]

    def _get_item_on(self,  block_id,  item_id,  what):

        if block_id not in self.cache or item_id not in self.cache[block_id]["items"]:
            return None

        item_id_on = self.cache[block_id]["items"][item_id][what]

        # TODO make constant in msg for it
        if item_id_on == 0:

            next_block_id = self.cache[block_id][what]

            if next_block_id == 0:

                return (0,  0)  # end of program

            items_dict = self.cache[next_block_id]["items"]
            next_item_id = min(items_dict,  key=items_dict.get)

            return (next_block_id, next_item_id)

        else:

            return (block_id,  item_id_on)

    def get_id_on_success(self,  block_id,  item_id):

        return self._get_item_on(block_id,  item_id,  "on_success")

    def get_id_on_failure(self,  block_id,  item_id):

        return self._get_item_on(block_id,  item_id,  "on_failure")

    def get_item_type(self, block_id, item_id):

        msg = self.get_item_msg(block_id, item_id)
        return msg.type

    def item_requires_learning(self, block_id, item_id):

        return self.get_item_type(block_id,  item_id) in [ProgramItem.MANIP_PICK, ProgramItem.MANIP_PLACE, ProgramItem.MANIP_PICK_PLACE]

    def is_place_pose_set(self, block_id, item_id):

        msg = self.get_item_msg(block_id, item_id)
        return msg.place_pose.pose.position.x != 0 and msg.place_pose.pose.position.y != 0

    def is_pick_pose_set(self, block_id, item_id):

        msg = self.get_item_msg(block_id, item_id)
        return msg.pick_pose.pose.position.x != 0 and msg.pick_pose.pose.position.y != 0

    def is_object_set(self, block_id, item_id):

        msg = self.get_item_msg(block_id, item_id)
        return msg.object != ""

    def is_pick_polygon_set(self, block_id, item_id):

        msg = self.get_item_msg(block_id, item_id)
        return len(msg.pick_polygon.polygon.points) > 0

    def is_place_polygon_set(self, block_id, item_id):

        msg = self.get_item_msg(block_id, item_id)
        return len(msg.place_polygon.polygon.points) > 0

    def program_learned(self):

        blocks = self.get_block_ids()

        for block_id in blocks:

            items = self.get_items_ids(block_id)

            for item_id in items:

                if self.item_learned(block_id, item_id) is False:
                    return False

        return True

    def item_learned(self, block_id, item_id):

        if not self.item_requires_learning(block_id,  item_id):
            return None

        msg = self.get_item_msg(block_id, item_id)

        # TODO other types
        if msg.type == ProgramItem.MANIP_PICK_PLACE:

            if not self.is_object_set(block_id, item_id):  # there has to be id or type
                return False

            if self.is_place_pose_set(block_id, item_id) or self.is_place_polygon_set(block_id, item_id):
                return True
            else:
                return False

            if msg.spec == ProgramItem.MANIP_ID:
                return True

            elif msg.spec == ProgramItem.MANIP_TYPE:

                if self.is_pick_polygon_set(block_id, item_id) or self.is_pick_pose_set(block_id, item_id):
                    return True
                else:
                    return False

        raise NotImplementedError("Not yet supported item type.")
