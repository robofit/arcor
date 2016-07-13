/*
 * Developed by dcgm-robotics@FIT group
 * Author: Michal Kapinus
 * Date: 01.04.2012 (version 0.1)
 *
 * License: BUT OPEN SOURCE LICENSE
 *------------------------------------------------------------------------------
 */

#pragma once
#ifndef _BUT_KINECT_NODE_
#define _BUT_KINECT_NODE_


#include <ros/ros.h> // Main header of ROS
#include <tf/transform_listener.h>

 #include <actionlib/client/simple_action_client.h>


#include <geometry_msgs/Pose.h>
#include <vector>
#include <iostream>
#include <string>
#include <visualization_msgs/Marker.h>
#include <tf/transform_broadcaster.h>
#include <art_msgs/UserStatus.h>
#include <art_msgs/UserActivity.h>


namespace art_table_pointing_kinect {

class ArtTablePointingKinect {
public:

    ArtTablePointingKinect();
    ~ArtTablePointingKinect();


    void process(std::string user_id);
    int whichSector(tf::Vector3);


    tf::TransformListener listener_;
    ros::Publisher markers_pub_;
	
    std::string table_frame_;

    int user_id;

private:

    ros::Publisher point_right_pub_, point_left_pub_, user_activity_pub_;
    ros::Subscriber user_status_sub_;

    void user_status(art_msgs::UserStatusConstPtr data);

    double table_width_, table_height_, x_offset_, y_offset_;
    
    tf::Vector3 computeIntersection(tf::StampedTransform point1, tf::StampedTransform point2,
                                    tf::StampedTransform plane, tf::Vector3 plane_normal);



    void visualizeArrow(tf::StampedTransform elbow, int id);
    void visualizeIntersection(tf::Vector3 point, int id);
    bool pointingAtTable(tf::Vector3 point, tf::StampedTransform elbow, tf::StampedTransform hand);
	
	ros::NodeHandle nh_;
    int current_marker_id_, current_calib_detections_;
	std::vector<geometry_msgs::Pose> markers_;
	ros::Timer timer_;
	tf::TransformBroadcaster br_;

    bool detecting_;
    bool show_arrows_, show_intersections_;
	
    void setActivity(int act);

    art_msgs::UserActivity act_;

};
}

#endif
