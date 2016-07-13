#include "art_table_pointing/kinect_node.h"

namespace art_table_pointing_kinect {

ArtTablePointingKinect::ArtTablePointingKinect()  {


    markers_pub_ = nh_.advertise<visualization_msgs::Marker>("visualization_marker", 10);
    point_right_pub_ = nh_.advertise<geometry_msgs::PoseStamped>("/art/user/pointing_right", 1);
    point_left_pub_ = nh_.advertise<geometry_msgs::PoseStamped>("/art/user/pointing_left", 1);
    user_activity_pub_ = nh_.advertise<art_msgs::UserActivity>("/art/user/activity", 1, true);

    user_status_sub_ = nh_.subscribe("/art/user/status", 1, &ArtTablePointingKinect::user_status, this);


    ros::NodeHandle nh("~");
    nh.param<double>("x_offset", x_offset_, -1);
    nh.param<double>("y_offset", y_offset_, -1);
    nh.param<double>("table_width", table_width_, -1);
    nh.param<double>("table_height", table_height_, -1);
    nh.param<std::string>("table_frame", table_frame_, "/marker");

    nh.param<bool>("show_arrows", show_arrows_, true);
    nh.param<bool>("show_intersections", show_intersections_, true);

    ROS_INFO_STREAM("x_offset: " << x_offset_ << " y_offset: " << y_offset_);
    ROS_INFO_STREAM("table_width: " << table_width_ << " table_height: " << table_height_);
    ROS_INFO_STREAM("table_frame: " << table_frame_.c_str());

    setActivity(art_msgs::UserActivity::UNKNOWN);

    user_id = 0;
    ROS_INFO_STREAM("Kinect pointing node initialized.");

}

ArtTablePointingKinect::~ArtTablePointingKinect() {

}

void ArtTablePointingKinect::setActivity(int act) {

    if (act == act_.activity) return;

    act_.activity = act;
    user_activity_pub_.publish(act_);

}

void ArtTablePointingKinect::user_status(art_msgs::UserStatusConstPtr data) {
    user_id = data->user_id;
}


/**
 * @brief ArtTablePointingKinect::computeIntersection Computes intersection between
 * line (constructed from two points) and plane (point and normal vector)
 * @param elbow
 * @param hand
 * @param table
 * @param plane_normal
 * @return
 */
tf::Vector3 ArtTablePointingKinect::computeIntersection(tf::StampedTransform point1, tf::StampedTransform point2,
                                                        tf::StampedTransform plane, tf::Vector3 plane_normal) {

    tf::Vector3 l;

    l.setX(point2.getOrigin().x() - point1.getOrigin().x());
    l.setY(point2.getOrigin().y() - point1.getOrigin().y());
    l.setZ(point2.getOrigin().z() - point1.getOrigin().z());

    tfScalar d = (plane.getOrigin() - point1.getOrigin()).dot(plane_normal);
    d /= l.dot(plane_normal);

    //tf::Quaternion rr = transform_elbow_right.getRotation() * d;
    return l * d + point1.getOrigin();

}

void ArtTablePointingKinect::visualizeArrow(tf::StampedTransform elbow, int id) {
    visualization_msgs::Marker marker;
    marker.type = marker.ARROW;
    marker.action = marker.ADD;
    marker.color.r = 1;
    marker.color.g = 0;
    marker.color.b = 0;
    marker.color.a = 1;
    marker.scale.x = 1.5;
    marker.scale.y = 0.01;
    marker.scale.z = 0.01;
    marker.pose.position.x = elbow.getOrigin().x();
    marker.pose.position.y = elbow.getOrigin().y();
    marker.pose.position.z = elbow.getOrigin().z();
    marker.pose.orientation.x = elbow.getRotation().x();
    marker.pose.orientation.y = elbow.getRotation().y();
    marker.pose.orientation.z = elbow.getRotation().z();
    marker.pose.orientation.w = elbow.getRotation().w();
    marker.header.frame_id = table_frame_;
    marker.header.stamp = ros::Time::now();
    marker.id = id;

    markers_pub_.publish(marker);
}

void ArtTablePointingKinect::visualizeIntersection(tf::Vector3 point, int id) {
    visualization_msgs::Marker marker;
    marker.action = marker.ADD;
    marker.type = marker.CUBE;
    marker.color.r = 0;
    marker.color.g = 1;
    marker.color.b = 0;
    marker.color.a = 1;
    marker.scale.x = 0.02;
    marker.scale.y = 0.02;
    marker.scale.z = 0.02;
    marker.pose.position.x = point.x();
    marker.pose.position.y = point.y();
    marker.pose.position.z = point.z();
    marker.pose.orientation.x = 0;
    marker.pose.orientation.y = 0;
    marker.pose.orientation.z = 0;
    marker.pose.orientation.w = 1;
    marker.header.frame_id = table_frame_;
    marker.header.stamp = ros::Time::now();
    marker.id = id;

    markers_pub_.publish(marker);
}

bool ArtTablePointingKinect::pointingAtTable(tf::Vector3 point, tf::StampedTransform elbow, tf::StampedTransform hand) {
    if (elbow.getOrigin().z() < hand.getOrigin().z()) {
        return false;
    }
    return !(point.x() < x_offset_ || point.x() > table_width_ + x_offset_ ||
             point.y() < y_offset_ || point.y() > table_height_ + y_offset_ );
}

void ArtTablePointingKinect::process(std::string user_id) {
    tf::StampedTransform transform_hand_right, transform_elbow_right, transform_hand_left,
            transform_elbow_left, transform_table;
    try {

        listener_.lookupTransform(table_frame_, "/left_hand_" + user_id, ros::Time(0), transform_hand_right);
        listener_.lookupTransform(table_frame_, "/left_elbow_" + user_id, ros::Time(0), transform_elbow_right);
        listener_.lookupTransform(table_frame_, "/right_hand_" + user_id, ros::Time(0), transform_hand_left);
        listener_.lookupTransform(table_frame_, "/right_elbow_" + user_id, ros::Time(0), transform_elbow_left);
        //listener_.lookupTransform(table_frame_, "/head_1", ros::Time(0), transform_head_right);
       // node.listener_.lookupTransform(table_frame_, "/left_wrist_1", ros::Time(0), transform_head_right);
    }
    catch (tf::TransformException ex) {
        ROS_WARN("No transform %s", ex.what());
        ros::Duration(2).sleep();
        return;
    }



    transform_table.setOrigin(tf::Vector3(0,0,0));
    tf::Vector3 table_normal;
    table_normal.setZ(1);
    table_normal.setX(0);
    table_normal.setY(0);

    tf::Vector3 point_right, point_left;
    point_right = computeIntersection(transform_elbow_right, transform_hand_right, transform_table, table_normal);
    point_left = computeIntersection(transform_elbow_left, transform_hand_left, transform_table, table_normal);

    if (show_arrows_) {
        visualizeArrow(transform_elbow_right, 0);
        visualizeArrow(transform_elbow_left, 1);
    }

    if (show_intersections_) {
        visualizeIntersection(point_right, 2);
        visualizeIntersection(point_left, 3);
    }

    geometry_msgs::PoseStamped pose;

    pose.pose.position.z = 0;
    pose.pose.orientation.w = 1;
    pose.header.stamp = ros::Time::now();
    pose.header.frame_id = table_frame_;

    // we want to set activity based on distance between closer hand and middle of the table
    // TODO what about using distance to object (the closest one) instead of middle of the table?
    tf::Vector3 middle_of_the_table(-table_width_/2.0, -table_height_/2.0, 0.0); // TODO +/- ??
    float min_dist = std::min(transform_hand_left.getOrigin().distance(middle_of_the_table), transform_hand_right.getOrigin().distance(middle_of_the_table));

    // TODO make thresholds configurable
    if (min_dist < 0.3) setActivity(art_msgs::UserActivity::WORKING);
    else if (min_dist > 0.5 && min_dist < 1.0) setActivity(art_msgs::UserActivity::READY);
    else if (min_dist > 1.2) setActivity(art_msgs::UserActivity::AWAY);

    if (pointingAtTable(point_right, transform_elbow_right, transform_hand_right)) {
        pose.pose.position.x = point_right.x();
        pose.pose.position.y = point_right.y();
        point_right_pub_.publish(pose);

    }
    if (pointingAtTable(point_left, transform_elbow_left, transform_hand_left)) {
        pose.pose.position.x = point_left.x();
        pose.pose.position.y = point_left.y();
        point_left_pub_.publish(pose);
    }

}


}

/* =============================================================================
 * Main function
 */
int main(int argc, char **argv) {
	// ROS initialization
    ros::init(argc, argv, "art_table_pointing_kinect");

    art_table_pointing_kinect::ArtTablePointingKinect node;

    ros::NodeHandle nh;
    ros::Rate r(60);
    while (nh.ok()) {
        if (node.user_id > 0) {
            node.process(std::to_string(node.user_id));
        }

        ros::spinOnce();
        r.sleep();
    }
	return 0;
}
