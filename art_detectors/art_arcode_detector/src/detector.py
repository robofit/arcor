#!/usr/bin/env python

from ar_track_alvar_msgs.msg import AlvarMarkers, AlvarMarker
from art_object_recognizer_msgs.msg import ObjInstance, InstancesArray
from shape_msgs.msg import SolidPrimitive
import sys
import rospy
import dataset


class ArCodeDetector:
    objects_db = None
    objects_table = None

    def __init__(self):
        self.objects_db = dataset.connect('sqlite:////home/artable/objdatabase.db')
        self.objects_table = self.objects_db['objects']
        #self.objects_table.insert(dict(name="juice", model_url="blablabla", obj_id=14))
        #self.objects_table.insert(dict(name="tea", model_url="blablabla", obj_id=1))
        # self.objects_table.update(dict(name="juice", model_url="blablabla", obj_id=0), ['name'])
        self.ar_code_sub = rospy.Subscriber("ar_pose_marker", AlvarMarkers, self.ar_code_cb)
        self.detected_objects_pub = rospy.Publisher("/art_object_detector/object", InstancesArray, queue_size=10)
        for ar in self.objects_table:
            print ar
        while not rospy.is_shutdown():
            pass

    def ar_code_cb(self, data):
        rospy.logdebug("New arcodes arrived:")
        instances = InstancesArray()
        for arcode in data.markers:
            obj = self.objects_table.find_one(obj_id=int(arcode.id))
            # rospy.logdebug(object if object is not None else "Object not in database")
            if obj is not None:
                rospy.logdebug(obj)
                obj_in = ObjInstance()
                obj_in.object_id = str(obj['name'])
                obj_in.pose = arcode.pose.pose
                obj_in.bbox.dimensions = [0.05, 0.05, 0.1]
                obj_in.bbox.type = SolidPrimitive.BOX

                instances.header.frame_id = "/marker"
                instances.instances.append(obj_in)

            else:
                rospy.logdebug("Object not in database")

        if len(data.markers) == 0:
            rospy.logdebug("Empty")
        else:
            self.detected_objects_pub.publish(instances)


if __name__ == '__main__':
    rospy.init_node('art_arcode_detector')
    # rospy.init_node('art_arcode_detector', log_level=rospy.DEBUG)
    try:
        node = ArCodeDetector()
    except rospy.ROSInterruptException:
        pass
