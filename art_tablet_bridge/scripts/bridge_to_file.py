#!/usr/bin/env python
import rospy
import jsonpickle
from std_msgs.msg import Int64
from geometry_msgs.msg import Pose, Point, Quaternion


class BridgeToFile:

    def __init__(self):
        rospy.Subscriber("/art_object_detector/object", Int64, self.callback)
        rospy.spin()

    def callback(self, detected_object):
        """
        :type detected_object: Int64
        :return:
        """
        rospy.loginfo(rospy.get_caller_id() + ": I Heard " + str(detected_object.data))
        detected = {'header': {'timestamp': 0, 'frame_id': 'dont_care', 'seq': 0}, 'objects': [
            {'object_id': 'obj1', 'pose': Pose(Point(1, 2, 3), Quaternion(0, 0, 0, 1))},
            {'object_id': 'obj2', 'pose': Pose(Point(4, 25, 6), Quaternion(1, 2, 0, 1))},
            {'object_id': 'obj3', 'pose': Pose(Point(8, 2, 4), Quaternion(0, 8, 0, 1))}
        ]}

        objects = []
        for obj in detected['objects']:
            to_json = {'object_id': obj['object_id'], 'pose': {'position': {'x': obj['pose'].position.x,
                                                                            'y': obj['pose'].position.y,
                                                                            'z': obj['pose'].position.z},
                                                               'orientation': {'x': obj['pose'].orientation.x,
                                                                               'y': obj['pose'].orientation.y,
                                                                               'z': obj['pose'].orientation.z,
                                                                               'w': obj['pose'].orientation.w}}}
            objects.append(to_json)
        # rospy.loginfo(jsonpickle.encode({'objects': objects}))
        with open('/home/ikapinus/objects.json', 'w') as f:
            f.write(jsonpickle.encode({'objects': objects}))




if __name__ == '__main__':
    rospy.init_node('bridge_to_file')

    try:
        node = BridgeToFile()
    except rospy.ROSInterruptException:
        pass
