#!/usr/bin/env python
import rospy
import jsonpickle
from art_msgs.msg import InstancesArray, ObjInstance
from std_msgs.msg import String


class BridgeToFile:

    marker_scale = 0
    marker_size = 6.52 #cm

    def __init__(self):
        rospy.Subscriber("/art_object_detector/object", InstancesArray, self.callback)
        self.pub = rospy.Publisher("/objects_string", String, queue_size=1)
        self.marker_scale = 100 / self.marker_size
        rospy.spin()

    def callback(self, detected_object):
        """
        :type detected_object: InstancesArray
        :return:
        """

        objects = []
        for obj in detected_object.instances:
            '''to_json = {'name': obj.object_id, 'pose': {'position': {'x': obj.pose.position.x,
                                                                         'y': obj.pose.position.y,
                                                                         'z': 0},
                                                            'orientation': {'x': obj.pose.orientation.x,
                                                                            'y': obj.pose.orientation.y,
                                                                            'z': obj.pose.orientation.z,
                                                                            'w': obj.pose.orientation.w}}}'''
            to_json = {'name': obj.object_id, 'position': {'x': obj.pose.position.x * self.marker_scale,
                                                                         'y': obj.pose.position.y * self.marker_scale,
                                                                         'z': 0},
                                                            'orientation': {'x': obj.pose.orientation.x,
                                                                            'y': obj.pose.orientation.y,
                                                                            'z': obj.pose.orientation.z,
                                                                            'w': obj.pose.orientation.w}}
            objects.append(to_json)
        #rospy.loginfo(jsonpickle.encode({'objects': objects}))
        #with open('/home/ikapinus/www/examplewww.json', 'w') as f:
        with open('/var/www/html/examplewww.json', 'w') as f:
            f.write(jsonpickle.encode(objects))
        




if __name__ == '__main__':
    rospy.init_node('bridge_to_file')

    try:
        node = BridgeToFile()
    except rospy.ROSInterruptException:
        pass
