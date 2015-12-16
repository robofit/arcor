#!/usr/bin/env python
import rospy
import jsonpickle
from std_msgs.msg import Int64
from geometry_msgs.msg import Pose, Point, Quaternion
from object_recognizer_msgs.msg import InstancesArray, ObjInstance


class BridgeToFile:

    def __init__(self):
        rospy.Subscriber("/art_object_detector/object", InstancesArray, self.callback)
        rospy.spin()

    def callback(self, detected_object):
        """
        :type detected_object: InstancesArray
        :return:
        """

        objects = []
        for obj in detected_object.instances:
            to_json = {'object_id': obj.object_id, 'pose': {'position': {'x': obj.pose.position.x,
                                                                         'y': obj.pose.position.y,
                                                                         'z': obj.pose.position.z},
                                                            'orientation': {'x': obj.pose.orientation.x,
                                                                            'y': obj.pose.orientation.y,
                                                                            'z': obj.pose.orientation.z,
                                                                            'w': obj.pose.orientation.w}}}
            objects.append(to_json)
        rospy.loginfo(jsonpickle.encode({'objects': objects}))
        with open('/home/ikapinus/objects.json', 'w') as f:
            f.write(jsonpickle.encode({'objects': objects}))




if __name__ == '__main__':
    rospy.init_node('bridge_to_file')

    try:
        node = BridgeToFile()
    except rospy.ROSInterruptException:
        pass
