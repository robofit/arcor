#!/usr/bin/env python

import rospy
import tf
import rostopic
import rosservice
import subprocess
from art_utils import Topic, Service, Action, ArtAPI


class ArtWtf():

    def __init__(self):

        self.world_frame = rospy.get_param("world_frame", "marker")
        self.frames_to_check = rospy.get_param(
            "frames_to_check", ("odom_combined", "kinect2_link"))

        self.ntp_server = rospy.get_param("ntp_server", "192.168.104.10")
        self.machines_to_check = rospy.get_param(
            "machines_to_check", ("pcspanel3", "artable-n1", "artable-n2", "pr1027"))

    def check_transformations(self):

        tfl = tf.TransformListener()
        rospy.sleep(2)

        err = 0

        if not tfl.frameExists(self.world_frame):

            print("  ERROR: world frame '" + self.world_frame +
                  "' does not exist! Can't do any checks.")
            return False

        for frame in self.frames_to_check:

            if not tfl.frameExists(frame):

                print("  ERROR: frame '" + frame + "' does not exist!")
                err += 1
                continue

            if not tfl.canTransform(self.world_frame, frame, rospy.Time(0)):

                print("  ERROR: unable to transform between '" +
                      frame + "' and world frame.")
                err += 1
                continue

        return err == 0

    def check_time_synchronization(self):

        err = 0

        for machine in self.machines_to_check:

            ping_ret = subprocess.call(
                ["ping", "-c 1", machine], stdout=subprocess.PIPE,  stderr=subprocess.PIPE)

            if ping_ret != 0:

                print("  ERROR: can't ping machine '" + machine +
                      "'. Check if it is up and running.")
                err += 1
                continue

            p = subprocess.Popen(
                ["ssh", "artable@" + machine, "ntpdate -q " + self.ntp_server], stdout=subprocess.PIPE)
            ret = p.communicate()
            try:
                ofs = abs(float(ret[0].split('\n')[
                          0].split(',')[2].split(' ')[2]))
            except IndexError:

                print(
                    "  ERROR: failed to get time offset for machine '" + machine + "'.")
                err += 1
                continue

            if ofs > 0.1:

                print("  ERROR: time offset too high for machine '" +
                      machine + "'. TF may not work as expected.")
                err += 1

        return err == 0

    def check_api(self):

        if not rospy.has_param("/art/api"):
            print("  ERROR: API definition not loaded, please run 'roslaunch art_bringup upload_api_definition.launch' first.")
            return False

        api = ArtAPI()

        err = 0

        for group in api.groups:

            print "  Checking API group: " + group.name + "..."

            gerr = 0

            for it in group.api:

                if isinstance(it, Topic):

                    topic_type = rostopic.get_topic_type(it.name)[0]

                    if topic_type is None:

                        print("    ERROR: topic '" + it.name +
                              "' is probably not yet advertised.")
                        gerr += 1
                        continue

                    if topic_type != it.type:

                        print("    ERROR: topic '" + it.name + "' should be of type '" +
                              it.type + "' but it is '" + topic_type + "'.")
                        gerr += 1
                        continue

                    cls = rostopic.get_topic_class(it.name)[0]

                    try:
                        rospy.wait_for_message(it.name, cls, 1.0)
                    except rospy.ROSException:
                        print(
                            "    ERROR: no one publishing to the topic '" + it.name + "'.")
                        gerr += 1
                        continue

                elif isinstance(it, Service):

                    service_type = rosservice.get_service_type(it.name)

                    if service_type is None:

                        print("    ERROR: service '" +
                              it.name + "' does not exist.")
                        gerr += 1
                        continue

                    if service_type != it.type:

                        print("    ERROR: service '" + it.name + "' should be of type '" +
                              it.type + "' but it is '" + service_type + "'.")
                        gerr += 1
                        continue

                elif isinstance(it, Action):

                    pass

            if gerr == 0:
                print "  OK"
            else:
                print("  Found " + str(gerr) + " error(s).")
            print

            if gerr > 0:
                err += 1

        return err == 0


if __name__ == '__main__':

    try:

        rospy.init_node('art_wtf_node')

        node = ArtWtf()

        print("This script tries to find out what might be wrong with ARTable. You may also try to use 'roswtf' utility to find out general ROS issues.")
        print

        print("Checking time synchronization...")
        print
        if node.check_time_synchronization():
            print("Done")
        print

        print("Checking transformations...")
        print
        if node.check_transformations():
            print("Done")
        print

        print("Checking API...")
        print
        if node.check_api():
            print("Done")
        print

    except rospy.ROSInterruptException:
        pass
