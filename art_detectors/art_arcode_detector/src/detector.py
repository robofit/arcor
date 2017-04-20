#!/usr/bin/env python

from ar_track_alvar_msgs.msg import AlvarMarkers
from art_msgs.msg import ObjInstance, InstancesArray
import rospy
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point
from tf import transformations
import copy
from art_utils import ArtApiHelper


class ArCodeDetector:

    objects_table = None

    def __init__(self):

        self.ar_code_sub = rospy.Subscriber("/art/object_detector/n1/ar_pose_marker", AlvarMarkers, self.ar_code_cb)
        self.detected_objects_pub = rospy.Publisher("/art/object_detector/object", InstancesArray, queue_size=10)
        self.visualize_pub = rospy.Publisher("art/object_detector/visualize_objects", Marker, queue_size=10)

        # TODO make a timer clearing this cache from time to time
        self.objects_cache = {}
        
        self.art = ArtApiHelper()
        self.art.wait_for_api()

    def ar_code_cb(self, data):

        rospy.logdebug("New arcodes arrived:")
        instances = InstancesArray()
        id = 0

        for arcode in data.markers:

            aid = int(arcode.id)

            # list of allowed object ids
            # TODO load from param
            if aid not in [4, 5, 3, 21]: 
                continue

            if aid not in self.objects_cache:

                # TODO AR code ID / object type assignment should be done somewhere...
                # type "profile_20_60" is just for example (used in art_db/test_db.py)

                object_type = self.art.get_object_type("profile_20_60")

                if object_type is None:

                    # error or unknown object - let's ignore it
                    self.objects_cache[aid] = None
                    continue

                self.objects_cache[aid] = {'type': object_type.name,  'bb': object_type.bbox}

            # skip unknown objects
            if self.objects_cache[aid] is None:
                continue

            obj_in = ObjInstance()
            obj_in.object_id = str(aid)
            obj_in.object_type = self.objects_cache[aid]['type']
            obj_in.pose = arcode.pose.pose

            angles = transformations.euler_from_quaternion([obj_in.pose.orientation.x,
                                                            obj_in.pose.orientation.y,
                                                            obj_in.pose.orientation.z,
                                                            obj_in.pose.orientation.w])

            q = transformations.quaternion_from_euler(0, 0, angles[2])
            obj_in.pose.orientation.x = q[0]
            obj_in.pose.orientation.y = q[1]
            obj_in.pose.orientation.z = q[2]
            obj_in.pose.orientation.w = q[3]
            # print self.objects_cache[aid]['bb']
            obj_in.pose.position.z = float(self.objects_cache[aid]['bb'].dimensions[2]/2)
            self.show_rviz_bb(obj_in, arcode.id, self.objects_cache[aid]['bb'])
            # obj_in.pose.position.z *= -1.0

            instances.header.stamp = arcode.header.stamp
            instances.header.frame_id = arcode.header.frame_id
            instances.instances.append(obj_in)
            ++id


        if len(data.markers) == 0:
            rospy.logdebug("Empty")
        else:
            #print instances
            self.detected_objects_pub.publish(instances)

    def show_rviz_bb(self, obj_in, id, bb):
        '''

        :type obj: ObjInstance
        :return:
        '''
        obj = copy.deepcopy(obj_in)
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

        bbox_x = float(bb.dimensions[0]/2)
        bbox_y = float(bb.dimensions[1]/2)
        bbox_z = float(bb.dimensions[2])
        marker.points = [
            Point(- bbox_x,- bbox_y, - bbox_z/2),
            Point(+ bbox_x,- bbox_y, - bbox_z/2),
            Point(+ bbox_x,- bbox_y, - bbox_z/2),
            Point(+ bbox_x,+ bbox_y, - bbox_z/2),
            Point(+ bbox_x,+ bbox_y, - bbox_z/2),
            Point(- bbox_x,+ bbox_y, - bbox_z/2),
            Point(- bbox_x,+ bbox_y, - bbox_z/2),
            Point(- bbox_x,- bbox_y, - bbox_z/2),

            Point(- bbox_x,- bbox_y, - bbox_z/2),
            Point(- bbox_x,- bbox_y,  + bbox_z/2),
            Point(+ bbox_x,- bbox_y, - bbox_z/2),
            Point(+ bbox_x,- bbox_y,  + bbox_z/2),
            Point(+ bbox_x,+ bbox_y, - bbox_z/2),
            Point(+ bbox_x,+ bbox_y,  + bbox_z/2),
            Point(- bbox_x,+ bbox_y, - bbox_z/2),
            Point(- bbox_x,+ bbox_y,  + bbox_z/2),

            Point(- bbox_x,- bbox_y,  + bbox_z/2),
            Point(+ bbox_x,- bbox_y,  + bbox_z/2),
            Point(+ bbox_x,- bbox_y,  + bbox_z/2),
            Point(+ bbox_x,+ bbox_y,  + bbox_z/2),
            Point(+ bbox_x,+ bbox_y,  + bbox_z/2),
            Point(- bbox_x,+ bbox_y,  + bbox_z/2),
            Point(- bbox_x,+ bbox_y,  + bbox_z/2),
            Point(- bbox_x,- bbox_y,  + bbox_z/2),
        ]

        marker.header.frame_id = "/marker"

        self.visualize_pub.publish(marker)
        marker.pose.position.z += 0.02 + bbox_z /2
        marker.id = int(id+100)
        marker.type = marker.TEXT_VIEW_FACING
        marker.text = obj.object_id
        marker.scale.z = 0.02
        self.visualize_pub.publish(marker)

if __name__ == '__main__':
    rospy.init_node('art_arcode_detector2')
    # rospy.init_node('art_arcode_detector', log_level=rospy.DEBUG)
    try:
        rospy.wait_for_service('/art/db/object_type/get')
        ArCodeDetector()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
