#!/usr/bin/env python
import rospy
import sys
from art_utils import ArtApiHelper, ProgramHelper
import pygraphviz as pgv
from art_msgs.msg import ProgramItem
from cgi import escape


def get_node_name(block_id, item_id):

    return 'b' + str(block_id) + 'i' + str(item_id)


def get_type_string(item):

    if item.type == ProgramItem.GET_READY:

        return "GET_READY"

    elif item.type == ProgramItem.NOP:

        return "NOP"

    elif item.type == ProgramItem.PICK_FROM_FEEDER:

        return "PICK_FROM_FEEDER\nobject type: " + item.object[0]

    elif item.type == ProgramItem.PICK_FROM_POLYGON:

        return "PICK_FROM_POLYGON\nobject type: " + item.object[0]

    elif item.type == ProgramItem.PICK_OBJECT_ID:

        return "PICK_OBJECT_ID\nobject ID: " + item.object[0]

    elif item.type == ProgramItem.PLACE_TO_POSE:

        return "PLACE_TO_POSE\nobject from ID: " + ", ".join(map(str, item.ref_id))

    elif item.type == ProgramItem.WAIT_FOR_USER:

        return "Wait for user"

    elif item.type == ProgramItem.WAIT_UNTIL_USER_FINISHES:
        return "Wait until user finishes"

    elif item.type == ProgramItem.DRILL_POINTS:
        return "DRILL_POINTS\n" + str(len(item.pose)) + " holes"

    elif item.type == ProgramItem.PLACE_TO_GRID:
        return "PLACE_TO_GRID\n" + str(len(item.pose)) + " objects"


def main(args):

    if len(args) < 2:

        print "This script needs program id as argument."
        return

    rospy.init_node('plot_program', anonymous=True)

    art = ArtApiHelper()
    ph = ProgramHelper()

    prog = art.load_program(int(args[1]))

    if prog is None:

        print "Failed to load program"
        return

    if not ph.load(prog):

        print "Faulty program"
        return

    A = pgv.AGraph(label="<<B>Program ID: " + str(prog.header.id) + "<br/>" + escape(prog.header.name) + "</B>>", directed=True, strict=False)
    A.graph_attr['outputorder'] = 'edgesfirst'

    A.add_node("start", label="Program start")
    A.add_node("b0i0", label="Program end")

    A.add_edge("start", get_node_name(*ph.get_first_item_id()))

    for block_id in ph.get_block_ids():

        block_msg = ph.get_block_msg(block_id)
        ids = []

        for item_id in ph.get_items_ids(block_id):

            nn = get_node_name(block_id, item_id)
            item_msg = ph.get_item_msg(block_id, item_id)

            osn = get_node_name(*ph.get_id_on_success(block_id, item_id))
            ofn = get_node_name(*ph.get_id_on_failure(block_id, item_id))

            A.add_edge(nn, osn, color="green", constraint=True)
            A.add_edge(nn, ofn, color="red", constraint=True)

            for ref in item_msg.ref_id:

                A.add_edge(get_node_name(block_id, ref), nn, color="gray", style="dashed", key="ref_" + nn + "_" + get_node_name(block_id, ref))

            A.get_node(nn).attr['label'] = 'Item ID: ' + str(item_id) + '\n' + get_type_string(item_msg)
            A.get_node(nn).attr['shape'] = 'box'
            A.get_node(nn).attr['style'] = 'rounded'
            ids.append(nn)

        sg = A.subgraph(name="cluster_" + str(block_id), label="<<b>Group ID: " + str(block_id) + "<br/>" + escape(block_msg.name) + "</b>>", color="gray")
        sg.add_nodes_from(ids)

    A.layout(prog='dot')
    A.draw('output.pdf')


if __name__ == '__main__':
    try:
        main(sys.argv)
    except KeyboardInterrupt:
        print("Shutting down")
