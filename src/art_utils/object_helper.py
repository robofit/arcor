import rospy
from art_msgs.msg import InstancesArray


class ObjectHelper(object):

    OBJECT_ADDED = 0
    OBJECT_UPDATED = 1
    OBJECT_LOST = 2

    def __init__(self, callback):

        self.callback = callback

        self.objects = {}

        self.objects_sub = rospy.Subscriber(
            '/art/object_detector/object_filtered', InstancesArray, self.objects_cb, queue_size=1)

    def objects_cb(self, msg):

        detected_objects = {}

        for inst in msg.instances:

            detected_objects[inst.object_id] = inst

            if inst.object_id not in self.objects:

                self.callback(self.OBJECT_ADDED, msg.header, inst)

            else:

                self.callback(self.OBJECT_UPDATED, msg.header, inst)

            self.objects[inst.object_id] = inst

        for k,v in self.objects.iteritems():

            if k not in detected_objects:

                self.callback(self.OBJECT_LOST, msg.header, v)

    def filter_by_type(self, object_type):

        return {k: v for k, v in self.objects.iteritems() if v.object_type == object_type}

