#!/usr/bin/env python
import rospy
import sys
from art_utils import ArtApiHelper, ProgramHelper
import pygraphviz as pgv
from art_msgs.msg import ProgramItem
from cgi import escape


def get_node_name(block_id,  item_id):

    return 'b' + str(block_id) + 'i' + str(item_id)


def get_type_string(item):

    if item.type == ProgramItem.GET_READY:

        return "Robot ready"

    elif item.type == ProgramItem.MANIP_PICK:

        if item.spec == ProgramItem.MANIP_ID:
            return "Pick ID: " + item.object
        elif item.spec == ProgramItem.MANIP_TYPE:
            return "Pick type: " + item.object

    elif item.type == ProgramItem.MANIP_PLACE:

        if item.spec == ProgramItem.MANIP_ID:
            return "Place ID: " + item.object
        elif item.spec == ProgramItem.MANIP_TYPE:
            return "Place type: " + item.object

    elif item.type == ProgramItem.MANIP_PICK_PLACE:

        if item.spec == ProgramItem.MANIP_ID:
            return "P&p ID: " + item.object
        elif item.spec == ProgramItem.MANIP_TYPE:
            return "P&p type: " + item.object

    elif item.type == ProgramItem.WAIT:

        if item.spec == ProgramItem.WAIT_FOR_USER:
            return "Wait for user"
        elif item.spec == ProgramItem.WAIT_UNTIL_USER_FINISHES:
            return "Wait until user finishes"


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

    A = pgv.AGraph(label="<<B>Program ID: " + str(prog.header.id) + "<br/>" + escape(prog.header.name) + "</B>>",  directed=True)
    A.graph_attr['outputorder'] = 'edgesfirst'

    A.add_node("start",  label="Program start")
    A.add_node("b0i0",  label="Program end")

    A.add_edge("start",  get_node_name(*ph.get_first_item_id()))

    for block_id in ph.get_block_ids():

        block_msg = ph.get_block_msg(block_id)
        ids = []

        for item_id in ph.get_items_ids(block_id):

            nn = get_node_name(block_id, item_id)
            item_msg = ph.get_item_msg(block_id,  item_id)

            A.add_edge(nn,  get_node_name(*ph.get_id_on_success(block_id,  item_id)),  label="on_success",  color="green")
            A.add_edge(nn,  get_node_name(*ph.get_id_on_failure(block_id,  item_id)),  label="on_failure",  color="red")
            A.get_node(nn).attr['label'] = 'Item ID: ' + str(item_id) + '\n' + get_type_string(item_msg)
            A.get_node(nn).attr['shape'] = 'box'
            A.get_node(nn).attr['style'] = 'rounded'
            ids.append(nn)

        A.subgraph(nbunch=ids, name="cluster_" + str(block_id),  label="<<b>Group ID: " + str(block_id) + "<br/>" + escape(block_msg.name) + "</b>>",  color="gray")

    A.layout(prog='dot')
    A.draw('output.pdf')


if __name__ == '__main__':
    try:
        main(sys.argv)
    except KeyboardInterrupt:
        print("Shutting down")
