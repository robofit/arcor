#!/usr/bin/env python

from ar_track_alvar_msgs.msg import AlvarMarkers, AlvarMarker
from art_msgs.msg import ObjInstance, InstancesArray
from shape_msgs.msg import SolidPrimitive
import sys
import rospy
import dataset
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point
from tf import transformations


class ArCodeDetector:
    objects_db = None
    objects_table = None

    def __init__(self):
        self.objects_db = dataset.connect('sqlite:////home/ikapinus/objdatabase.db')
        self.objects_table = self.objects_db['objects']
        # self.objects_table.insert(dict(name="profile_3", model_url="blablabla", obj_id=16))
        # self.objects_table.insert(dict(name="profile_2", model_url="blablabla", obj_id=16))
        # self.objects_table.update(dict(name="profile_3", model_url="blablabla", obj_id=15), ['name'])
        self.ar_code_sub = rospy.Subscriber("ar_pose_marker", AlvarMarkers, self.ar_code_cb)
        self.detected_objects_pub = rospy.Publisher("/art_object_detector/object", InstancesArray, queue_size=10)
        self.visualize_pub = rospy.Publisher("art_object_detector/visualize_objects", Marker, queue_size=10)
        for ar in self.objects_table:
            print ar

    def ar_code_cb(self, data):
        rospy.logdebug("New arcodes arrived:")
        instances = InstancesArray()
        id = 0
        for arcode in data.markers:
            obj = self.objects_table.find_one(obj_id=int(arcode.id))
            # rospy.logdebug(object if object is not None else "Object not in database")
            if obj is not None:
                rospy.logdebug(obj)
                obj_in = ObjInstance()
                obj_in.object_id = str(obj['name'])
                obj_in.pose = arcode.pose.pose
                obj_in.pose.position.z = 0

                angles = transformations.euler_from_quaternion([obj_in.pose.orientation.x,
                                                                obj_in.pose.orientation.y,
                                                                obj_in.pose.orientation.z,
                                                                obj_in.pose.orientation.w])

                q = transformations.quaternion_from_euler(0, 0, angles[2])
                obj_in.pose.orientation.x = q[0]
                obj_in.pose.orientation.y = q[1]
                obj_in.pose.orientation.z = q[2]
                obj_in.pose.orientation.w = q[3]

                obj_in.bbox.dimensions = [obj['bbox_x'], obj['bbox_y'], obj['bbox_z']]
                obj_in.bbox.type = SolidPrimitive.BOX
                self.show_rviz_bb(obj_in, arcode.id)
                instances.header.stamp = arcode.header.stamp
                instances.header.frame_id = arcode.header.frame_id
                instances.instances.append(obj_in)
                ++id

            else:
                rospy.logdebug("Object not in database")

        if len(data.markers) == 0:
            rospy.logdebug("Empty")
        else:
            self.detected_objects_pub.publish(instances)

    def show_rviz_bb(self, obj, id):
        '''

        :type obj: ObjInstance
        :return:
        '''
        marker = Marker()
        marker.type = marker.LINE_LIST
        marker.id = int(id)
        marker.action = marker.ADD
        marker.scale.x = 0.001
        marker.scale.y = 0.01
        marker.scale.z = 0.01
        marker.color.r = 0
        marker.color.g = 1
        marker.color.b = 0
        marker.color.a = 1
        marker.lifetime = rospy.Duration(5)
        marker.pose = obj.pose


        pos = obj.pose.position

        bbox_x = float(obj.bbox.dimensions[0]/2)
        bbox_y = float(obj.bbox.dimensions[1]/2)
        bbox_z = float(obj.bbox.dimensions[2])
        marker.points = [
            Point(- bbox_x,- bbox_y, 0),
            Point(+ bbox_x,- bbox_y, 0),
            Point(+ bbox_x,- bbox_y, 0),
            Point(+ bbox_x,+ bbox_y, 0),
            Point(+ bbox_x,+ bbox_y, 0),
            Point(- bbox_x,+ bbox_y, 0),
            Point(- bbox_x,+ bbox_y, 0),
            Point(- bbox_x,- bbox_y, 0),

            Point(- bbox_x,- bbox_y, 0),
            Point(- bbox_x,- bbox_y,  + bbox_z),
            Point(+ bbox_x,- bbox_y, 0),
            Point(+ bbox_x,- bbox_y,  + bbox_z),
            Point(+ bbox_x,+ bbox_y, 0),
            Point(+ bbox_x,+ bbox_y,  + bbox_z),
            Point(- bbox_x,+ bbox_y, 0),
            Point(- bbox_x,+ bbox_y,  + bbox_z),

            Point(- bbox_x,- bbox_y,  + bbox_z),
            Point(+ bbox_x,- bbox_y,  + bbox_z),
            Point(+ bbox_x,- bbox_y,  + bbox_z),
            Point(+ bbox_x,+ bbox_y,  + bbox_z),
            Point(+ bbox_x,+ bbox_y,  + bbox_z),
            Point(- bbox_x,+ bbox_y,  + bbox_z),
            Point(- bbox_x,+ bbox_y,  + bbox_z),
            Point(- bbox_x,- bbox_y,  + bbox_z),
        ]

        marker.header.frame_id = "/marker"

        self.visualize_pub.publish(marker)
        marker.pose.position.z += 0.02 + bbox_z
        marker.id = int(id+100)
        marker.type = marker.TEXT_VIEW_FACING
        marker.text = obj.object_id
        marker.scale.z = 0.02
        self.visualize_pub.publish(marker)

if __name__ == '__main__':
    rospy.init_node('art_arcode_detector')
    # rospy.init_node('art_arcode_detector', log_level=rospy.DEBUG)
    try:
        node = ArCodeDetector()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
