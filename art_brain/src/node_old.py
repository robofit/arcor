#!/usr/bin/env python

import rospy
import time

import actionlib
from art_msgs.msg import LocalizeAgainstUMFAction, LocalizeAgainstUMFGoal, LocalizeAgainstUMFResult
from std_srvs.srv import Empty, EmptyRequest, EmptyResponse
from art_msgs.msg import UserStatus
from geometry_msgs.msg import PoseStamped, Pose
from std_msgs.msg import String
from art_msgs.msg import pickplaceAction, pickplaceGoal, SystemState


class ArtBrain:
    UNKNOWN = -1  # should not happen!
    START = 0
    CALIBRATING = 1
    NO_USER = 2
    WAITING_FOR_OBJECT = 3
    TRY_PICK = 4
    PICKING = 5
    WAITING_FOR_PLACE = 6
    TRY_PLACE = 7
    PLACING = 8
    DONE = 9
    BACK_TO_INIT_PLACE = 10
    BACK_TO_INIT_PICK = 11

    def __init__(self):
        self.show_marker_service = rospy.get_param('show_marker_service', '/art_simple_gui/show_marker')
        self.hide_marker_service = rospy.get_param('hide_marker_service', '/art_simple_gui/hide_marker')
        self.table_localize_action = rospy.get_param('table_localize_action', '/umf_localizer_node_table/localize')
        self.pr2_localize_action = rospy.get_param('pr2_localize_action', '/umf_localizer_node/localize')

        self.calibrate_pr2 = rospy.get_param('calibrate_pr2', True)
        self.calibrate_table = rospy.get_param('calibrate_table', True)

        self.user_status_sub = rospy.Subscriber("/art_table_pointing/user_status", UserStatus, self.user_status_cb)
        self.object_to_pick_sub = rospy.Subscriber("/art_simple_gui/selected_object", UserStatus, self.user_status_cb)
        self.pose_to_place_sub = rospy.Subscriber("/art_simple_gui/selected_place", UserStatus, self.user_status_cb)

        self.state_publisher = rospy.Publisher("/art_brain/system_state", SystemState, queue_size=1)

        self.pp_client = actionlib.SimpleActionClient('/pr2_pick_place_left/pp', pickplaceAction)

        self.state = self.START
        self.user_id = 0
        self.selected_object = None  # type: str
        self.selected_object_last_update = None  # type: rospy.Time
        self.selected_place = None  # type: PoseStamped

    def user_status_cb(self, data):
        '''

        :type data: UserStatus
        :return:
        '''
        self.user_id = data.user_id
        pass

    def selected_object_cb(self, data):
        '''

        :type data: String
        :return:
        '''
        self.selected_object = data.data
        self.selected_object_last_update = rospy.get_rostime()

    def selected_place_cb(self, data):
        '''

        :type data: PoseStamped
        :return:
        '''
        self.selected_place = data

    def check_user_active(self):
        return self.user_id != 0

    def calibrate(self, action_name, server="unknown", timeout=5):
        client = actionlib.SimpleActionClient(action_name, LocalizeAgainstUMFAction)
        rospy.logdebug("Waiting for server (" + server + ")")
        client.wait_for_server()
        rospy.logdebug("Server ready (" + server + ")")
        goal = LocalizeAgainstUMFGoal()
        goal.timeout = rospy.Duration(timeout)
        rospy.logdebug("Sending goal to server (" + server + ")")
        client.send_goal(goal)
        rospy.logdebug("Waiting for results  (" + server + ")")
        client.wait_for_result()
        return not client.get_result().result
        pass

    def calibrate_all(self, table_calibration=True, pr2_calibration=True):
        rospy.loginfo("Starting calibrating process")
        rospy.logdebug("Waiting for service " + self.show_marker_service)
        rospy.wait_for_service(self.show_marker_service)
        try:
            show_marker = rospy.ServiceProxy(self.show_marker_service, Empty)
            show_marker()

        except rospy.ServiceException, e:
            rospy.logerr("Service call failed: %s"%e)
            return False

        while table_calibration:
            if self.calibrate(self.table_localize_action, "table", 5):
                table_calibration = False
                rospy.loginfo("Table successfully calibrated")
            else:
                rospy.logwarn("Table calibration failed! Trying every 5 sec")
                time.sleep(5)
        while pr2_calibration:
            if self.calibrate(self.pr2_localize_action, "pr2", 5):
                pr2_calibration = False
                rospy.loginfo("PR2 successfully calibrated")
            else:
                rospy.logwarn("PR2 calibration failed! Trying every 5 sec")
                time.sleep(5)
        rospy.loginfo("Calibration done, hiding umf marker")
        rospy.logdebug("Waiting for service " + self.hide_marker_service)
        rospy.wait_for_service(self.hide_marker_service)
        try:
            hide_marker = rospy.ServiceProxy(self.hide_marker_service, Empty)
            hide_marker()

        except rospy.ServiceException, e:
            rospy.logerr("Service call failed: %s"%e)
            return False

        pass

    def state_unknown(self):
        pass

    def state_start(self):
        if self.calibrate_pr2 or self.calibrate_table:
                self.state = self.CALIBRATING

    def state_calibrating(self):
        self.calibrate_all(self.calibrate_table, self.calibrate_pr2)
        self.state = self.NO_USER

    def state_no_user(self):
        if self.check_user_active():
            self.state = self.WAITING_FOR_OBJECT

    def state_waiting_for_object(self):
        if self.selected_object is not None:
            self.state = self.TRY_PICK

    def state_try_pick(self):
        # TODO: check if everything is OK

        self.state = self.PICKING

    def state_picking(self):
        if self.pick_object(self.selected_object):
            self.state = self.WAITING_FOR_PLACE
            rospy.logdebug("Picking successfully done")
            self.selected_object = None
            self.selected_object_last_update = None
        else:
            self.state = self.BACK_TO_INIT_PICK

    def state_waiting_for_place(self):
        if self.selected_place is not None:
            self.state = self.TRY_PLACE

    def state_try_place(self):
        # TODO: check if everything is OK
        self.state = self.PLACING

    def state_placing(self):
        if self.place_object(self.selected_place.pose):
            self.state = self.DONE
            self.selected_place = None
            rospy.logdebug("Place successfully done.")
        else:
            self.state = self.BACK_TO_INIT_PLACE

    def state_pick_and_place_done(self):
        self.state = self.WAITING_FOR_OBJECT

    def state_back_to_init_place(self):
        self.selected_place = None
        self.state = self.WAITING_FOR_PLACE

    def state_back_to_init_pick(self):
        self.selected_object = None
        self.state = self.WAITING_FOR_OBJECT

    def state_switcher(self):
        states = {
            self.START: self.state_start,
            self.CALIBRATING: self.state_calibrating,
            self.NO_USER: self.state_no_user,
            self.WAITING_FOR_OBJECT: self.state_waiting_for_object,
            self.TRY_PICK: self.state_try_pick,
            self.PICKING: self.state_picking,
            self.WAITING_FOR_PLACE: self.state_waiting_for_place,
            self.TRY_PLACE: self.state_try_place,
            self.PLACING: self.state_placing,
            self.DONE: self.state_pick_and_place_done,
            self.BACK_TO_INIT_PLACE: self.state_back_to_init_place,
            self.BACK_TO_INIT_PICK: self.state_back_to_init_pick
        }
        return states.get(self.state, default=self.state_unknown)

    def show_umf_marker(self):
        pass

    def hide_umf_marker(self):
        pass

    def pick_object(self, object_id):
        '''

        :type object_id: str
        :return:
        '''
        goal = pickplaceGoal()
        goal.id = object_id
        goal.operation = goal.PICK
        goal.keep_orientation = False
        rospy.loginfo("Picking object with ID: " + str(object_id))
        self.pp_client.send_goal(goal)
        self.pp_client.wait_for_result()
        # TODO: make some error msg etc
        '''rospy.loginfo('got result')
        print self.pp_client.get_result()
        print "status: " + self.pp_client.get_goal_status_text()
        print "state: " + str(self.pp_client.get_state())
        '''
        if self.pp_client.get_result().result == 0:
            return True
        else:
            return False


    def place_object(self, obj, place):
        '''

        :type obj: str
        :type place: Pose
        :return:
        '''
        goal = pickplaceGoal()
        goal.operation = goal.PLACE
        goal.id = obj
        goal.place_pose = PoseStamped()

        goal.place_pose = place
        goal.place_pose.header.stamp = rospy.Time.now()
        # TODO: how to deal with this?
        goal.place_pose.pose.position.z = 0.74 + 0.06# + obj.bbox.dimensions[2]/2
        self.pp_client.send_goal(goal)
        self.pp_client.wait_for_result()
        if self.pp_client.get_result().result == 0:
            return True
        else:
            return False

    def publish_state(self):
        data = SystemState()
        data.state = self.state
        self.state_publisher.publish(data)

    def process(self):
        state_function = self.state_switcher()
        state_function()
        self.publish_state()


if __name__ == '__main__':
    rospy.init_node('art_brain')
    try:
        node = ArtBrain()
        rate = rospy.Rate(30)
        while not rospy.is_shutdown():
            node.process()
            rate.sleep()
    except rospy.ROSInterruptException:
        pass
