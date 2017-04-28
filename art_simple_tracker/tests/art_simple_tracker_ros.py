#!/usr/bin/env python

import unittest
import rostest
import sys
import rospy
import rospkg
from art_msgs.msg import UserStatus
from art_simple_tracker import ArtSimpleTracker
from std_msgs.msg import Header
from geometry_msgs.msg import Pose, Quaternion

rospack = rospkg.RosPack()


class TestSimpleTrackerRos(unittest.TestCase):

    def pose_equal(self, pose1, pose2, msg=None):
        rospy.logerr(pose1)
        rospy.logerr(pose2)
        if pose1.position.x != pose2.position.x or \
           pose1.position.y != pose2.position.y or \
           pose1.position.z != pose2.position.z or \
           pose1.orientation.x != pose2.orientation.x or \
           pose1.orientation.y != pose2.orientation.y or \
           pose1.orientation.z != pose2.orientation.z or \
           pose1.orientation.w != pose2.orientation.w:
            raise self.failureException()
        return

    def test_transform(self):
        tracker = ArtSimpleTracker("/marker")

        pose = Pose()
        pose.position.x = 0
        pose.position.y = 0
        pose.position.z = 0
        pose.orientation.w = 1

        header = Header()
        header.frame_id = "/camera"
        header.stamp = rospy.Time(0)
        pose = tracker.transform(header, pose)

        pose2 = Pose()

        pose2.position.x = 1.0
        pose2.orientation.w = 1.0
        self.addTypeEqualityFunc(Pose, self.pose_equal)
        self.assertEquals(pose.pose, pose2, "test_pose_equal")

   
if __name__ == '__main__':

    rospy.init_node('art_simple_tracker_test_node')
    # rospy.sleep(2.0)  # ...let init_db script does its work...
    rostest.run('art_simple_tracker', 'test_simple_tracker_ros',
                TestSimpleTrackerRos, sys.argv)
