#include <ros/ros.h>
// PCL specific includes
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/Image.h>

#include <tf/transform_listener.h>


#include <geometry_msgs/PointStamped.h>
#include <art_msgs/ObjectsCentroids.h>

#include <ar_track_alvar_msgs/AlvarMarkers.h>
#include <ar_track_alvar_msgs/AlvarMarker.h>
#include <geometry_msgs/Pose.h>
#include <tf/tf.h>
#include <exception>
#include <tf/transform_broadcaster.h>

class NoMainMarker: public std::exception {
public:
    virtual const char* what() const throw() {
        return "No main marker!";
    }
};


class ArtCalibration {
public:
    ArtCalibration() {
        table_marker_sub = nh_.subscribe ("/table/ar_pose_marker", 1, &ArtCalibration::table_marker_cb, this);
        pr2_marker_sub = nh_.subscribe ("/pr2/ar_pose_marker", 1, &ArtCalibration::pr2_marker_cb, this);
        ros_init();
    }

    ~ArtCalibration() {

    }

    void ros_init() {
        nh_.param<std::string>("robot_frame", robot_frame_, "odom_combined");
        nh_.param<std::string>("table_frame", table_frame_, "kinect2_link");
        nh_.param<std::string>("world_frame", world_frame_, "table");
    }

private:

    tf::StampedTransform tr_table_, tr_pr2_;
    tf::TransformBroadcaster br_;
    ros::Timer tr_timer_;

    ros::Subscriber table_marker_sub, pr2_marker_sub;
    ros::NodeHandle nh_;

    std::string world_frame_, robot_frame_, table_frame_;

    bool table_calibration_done_ = false, pr2_calibration_done_ = false;

    static const int MAIN_MARKER_SIZE = 10;


    void table_marker_cb(ar_track_alvar_msgs::AlvarMarkersConstPtr markers) {
        geometry_msgs::Pose pose;
        try {
            pose = get_main_marker_pose(*markers);
        }
        catch (NoMainMarker& e) {
            std::cout << e.what() << std::endl;
            return;
        }
        tr_table_ = create_transform_from_pose(pose, table_frame_);
        table_marker_sub.shutdown();
        table_calibration_done_ = true;
        tr_timer_ = nh_.createTimer(ros::Duration(0.1), &ArtCalibration::trCallback, this);
    }

    void pr2_marker_cb(ar_track_alvar_msgs::AlvarMarkersConstPtr markers) {
        geometry_msgs::Pose pose;
        try {
            pose = get_main_marker_pose(*markers);
        }
        catch (NoMainMarker& e) {
            std::cout << e.what() << std::endl;
            return;
        }
        tr_pr2_ = create_transform_from_pose(pose, robot_frame_);
        pr2_marker_sub.shutdown();
        pr2_calibration_done_ = true;
        tr_timer_ = nh_.createTimer(ros::Duration(0.1), &ArtCalibration::trCallback, this);
    }

    tf::StampedTransform create_transform_from_pose(geometry_msgs::Pose pose, std::string output_frame) {
        tf::Transform tr;
        tf::Quaternion q;
        tr.setOrigin(tf::Vector3(pose.position.x, pose.position.y, pose.position.z));
        tf::quaternionMsgToTF(pose.orientation, q);
        q.normalize();
        tr.setRotation(q);
        return tf::StampedTransform(tr.inverse(), ros::Time::now(), world_frame_, output_frame);
    }

    geometry_msgs::Pose get_main_marker_pose(ar_track_alvar_msgs::AlvarMarkers marker) {
        for (int i = 0; i < marker.markers.size(); ++i) {
            if (marker.markers[i].id == MAIN_MARKER_SIZE) {
                return marker.markers[i].pose.pose;
            }
        }
        return geometry_msgs::Pose();
    }

    void trCallback(const ros::TimerEvent& event) {
        if (table_calibration_done_) {
            tr_table_.stamp_ = ros::Time::now();
            br_.sendTransform(tr_table_);
        }
        if (pr2_calibration_done_) {
            tr_pr2_.stamp_ = ros::Time::now();
            br_.sendTransform(tr_pr2_);
        }
    }

};


int main (int argc, char** argv)
{
  // Initialize ROS
  ros::init (argc, argv, "art_calibration");


  ArtCalibration calibration;

  // Spin
  ros::spin();

}
