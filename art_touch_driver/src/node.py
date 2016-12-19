#!/usr/bin/env python

import rospy
import time
import numpy as np

import pycopia.OS.Linux.Input as input

from geometry_msgs.msg import PointStamped
from art_msgs.msg import Touch


class Slot:

    def __init__(self, slot_id=None, track_id=None):
        if slot_id is None:
            self.slot_id = -1
        else:
            self.slot_id = slot_id

        if track_id is None:
            self.track_id = -1
        else:
            self.track_id = track_id

        self.x = 0
        self.y = 0

    def __eq__(self, other):
        return self.slot_id == other.slot_id


class ArtTouchDriver:

    def __init__(self):
        self.x = 0
        self.y = 0
        self.touch = False
        self.touch_id = -1
        self.device = input.EventDevice("/dev/input/event17")
        self.touch_pub = rospy.Publisher("/art/interface/touchtable/touch", Touch, queue_size=1)
        self.slots = []
        self.slot = None
        self.to_delete_id = -1

    def get_slot_by_id(self, slot_id):
        for slot in self.slots:
            if slot.slot_id == slot_id:
                return slot
        return None

    def process(self):
        # print 1 if self.device._eventq else 0
        event = self.device.read()
        print event
        while True:
            if event.evtype == 3 and event.code == 47 and event.value >= 0:
                # MT_SLOT
                self.slot = self.get_slot_by_id(event.value)
                if self.slot is None:
                    self.slot = Slot(slot_id=event.value)
                    self.slots.append(self.slot)

            elif event.evtype == 3 and event.code == 57 and event.value >= 0:
                # MT_TRACK_ID start
                if self.slot is None:
                    self.slot = Slot(track_id=event.value, slot_id=0)
                    self.slots.append(self.slot)
                else:
                    self.slot.track_id = event.value

            elif event.evtype == 3 and event.code == 57 and event.value < 0:
                # MT_TRACK_ID end
                if self.slot is not None:
                    self.to_delete_id = self.slot.track_id
                    self.slots.remove(self.slot)
                    self.slot = None

            elif event.evtype == 3 and event.code == 53:
                # x position
                self.slot.x = event.value
                print("x")

            elif event.evtype == 3 and event.code == 54:
                # y position
                self.slot.y = event.value
                print("y")

            elif event.evtype == 0:

                touch = Touch()
                if self.slot is None:
                    touch.id = self.to_delete_id
                    touch.touch = False
                else:
                    touch.touch = True
                    touch.id = self.slot.track_id
                    touch.point = PointStamped()
                    touch.point.point.x = self.slot.x
                    touch.point.point.y = self.slot.y

                self.touch_pub.publish(touch)

            else:
                pass
            # print event

            if not self.device._eventq:
                break
            event = self.device.read()

if __name__ == '__main__':
    rospy.init_node('art_touch_driver')

    rospy.loginfo('Waiting for other nodes to come up...')

    rospy.loginfo('Ready!')

    try:
        node = ArtTouchDriver()
        rate = rospy.Rate(1000)

        while not rospy.is_shutdown():
            node.process()
            rate.sleep()
    except rospy.ROSInterruptException:
        pass
