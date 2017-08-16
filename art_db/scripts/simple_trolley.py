#!/usr/bin/env python

import sys
import rospy
from art_msgs.msg import Program, ProgramBlock, ProgramItem
from copy import deepcopy
from art_utils import ArtApiHelper
from art_utils.art_msgs_functions import obj_type, wait_item, feeder_item, grid_item, drill_item, item


def main(args):

    rospy.init_node('simple_trolley_init_script', anonymous=True)

    art = ArtApiHelper()
    art.wait_for_api()

    # -------------------------------------------------------------------------------------------
    # Simplified trolley assembly: object types
    # -------------------------------------------------------------------------------------------

    art.store_object_type(obj_type("p40x40x200", 0.04, 0.04, 0.2))
    art.store_object_type(obj_type("p40x40x400", 0.04, 0.04, 0.4))
    art.store_object_type(obj_type("p40x40x500", 0.04, 0.04, 0.5))

    # -------------------------------------------------------------------------------------------
    # Simplified trolley assembly: program
    # -------------------------------------------------------------------------------------------

    prog = Program()
    prog.header.id = 20
    prog.header.name = "Montaz voziku (A)"

    # --- left side of the trolley ------------------------------------------------------
    pb = ProgramBlock()
    pb.id = 2
    pb.name = "Bocnice 1"
    pb.on_success = 3
    pb.on_failure = 0
    prog.blocks.append(pb)

    pb.items.append(wait_item(2, on_failure=2))

    # each side consists of four profiles (of two types)
    pb.items.append(feeder_item(3, obj_type="p40x40x400"))
    pb.items.append(grid_item(4, on_success=3, on_failure=5, ref_id=[3]))

    pb.items.append(feeder_item(5, obj_type="p40x40x200"))
    pb.items.append(grid_item(6, on_success=5, on_failure=11, ref_id=[5]))

    # after p&p, let's drill holes
    pb.items.append(drill_item(11, on_success=11, on_failure=12, ref_id=[4]))
    pb.items.append(drill_item(12, on_success=12, on_failure=13, ref_id=[6]))

    pb.items.append(item(13, ProgramItem.GET_READY, on_success=0, on_failure=13))

    # --- right side of the trolley ------------------------------------------------------
    pb = deepcopy(pb)
    pb.id = 3
    pb.name = "Bocnice 2"
    pb.on_success = 4
    pb.on_failure = 0
    prog.blocks.append(pb)

    # --- connecting profiles ------------------------------------------------------------
    pb = ProgramBlock()
    pb.id = 4
    pb.name = "Spojovaci profily"
    pb.on_success = 2
    pb.on_failure = 0
    prog.blocks.append(pb)

    pb.items.append(wait_item(1, on_success=10, on_failure=1))

    pb.items.append(feeder_item(10, obj_type="p40x40x200"))
    pb.items.append(grid_item(11, on_success=10, on_failure=12, ref_id=[10], objects=4))

    pb.items.append(item(12, ProgramItem.GET_READY, on_success=0, on_failure=12))

    art.store_program(prog)


if __name__ == '__main__':
    try:
        main(sys.argv)
    except KeyboardInterrupt:
        print("Shutting down")
