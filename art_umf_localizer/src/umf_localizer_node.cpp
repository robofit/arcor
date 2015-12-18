#include "umf_localizer_node.h"
#include <opencv2/core/eigen.hpp>
#include <sensor_msgs/image_encodings.h>
#include <cv_bridge/cv_bridge.h>
#include <eigen_conversions/eigen_msg.h>

using namespace umf_localizer_node;
using namespace umf;
using namespace std;

umfLocalizerNode::umfLocalizerNode(ros::NodeHandle& nh): it_(nh), as_(nh, "localize", false){

  nh_ = nh;
  detector_.reset(new umf::UMFDetector<1>(UMF_FLAG_ITER_REFINE| UMF_FLAG_TRACK_POS | UMF_FLAG_MAX_PRECISION | UMF_FLAG_SUBWINDOWS | UMF_FLAG_SUBPIXEL));
  detector_->setTrackingFlags(UMF_TRACK_MARKER | UMF_TRACK_SCANLINES);
  detector_->tracker.setSubSampling(umf::Tracker::SUBSAMPLE_1);
  detector_->model.setPnPFlags(PNP_FLAG_COMPUTE_CAMERA | PNP_FLAG_GL_PROJECTION_MV | PNP_FLAG_SWAP_Y | PNP_FLAG_RIGHT_HANDED | PNP_FLAG_FILTER_REPR | PNP_FLAG_LOOK_Z_POSITIVE);
  
  cam_info_sub_ = nh_.subscribe("/cam_info_topic", 1, &umfLocalizerNode::cameraInfoCallback, this);
  pose_pub_ = nh_.advertise<geometry_msgs::PoseStamped>("pose", 10);

  as_.registerGoalCallback(boost::bind(&umfLocalizerNode::goalCB, this));
  as_.registerPreemptCallback(boost::bind(&umfLocalizerNode::preemptCB, this));

  as_.start();

}

void umfLocalizerNode::goalCB() {

    localization_to_ = as_.acceptNewGoal()->timeout;
    localization_start_ = ros::Time::now();
    cam_image_sub_ = it_.subscribe("/cam_image_topic", 1, &umfLocalizerNode::cameraImageCallback, this);
    ROS_INFO("new goal");

}

void umfLocalizerNode::preemptCB() {

    cam_image_sub_.shutdown();
    as_.setPreempted();

}

bool umfLocalizerNode::init() {

  string marker;

  nh_.param("continuous_mode", continuous_mode_, false);

  if (nh_.hasParam("marker")) {
  
    nh_.param<std::string>("marker", marker, "");
    if (!detector_->loadMarker(marker.c_str())) {
    
      ROS_ERROR("Error on loading marker file!");
      return false;

    }
  
  } else if (nh_.hasParam("marker_xml")) {
  
    nh_.param<std::string>("marker_xml", marker, "");
    if (!detector_->loadMarkerXML(marker.c_str())) {
    
      ROS_ERROR("Error on loading marker XML file!");
      return false;
    
    }
  
  } else {
  
      ROS_ERROR("Marker (XML) file not specified!");
      return false;
  
  }
  
  nh_.param<std::string>("robot_frame", robot_frame_, "odom_combined");
  nh_.param<std::string>("world_frame", world_frame_, "marker");
  nh_.param<double>("square_size", square_size_, 0.1);
  
  ROS_INFO_STREAM("Ready! Robot frame: " << robot_frame_ << ", world frame: " << world_frame_ << ", square size: " << square_size_ << ".");
  if (continuous_mode_) ROS_INFO("Continuous mode.");
  else ROS_INFO("Action server mode.");
  return true;

} 

void umfLocalizerNode::cameraInfoCallback(const sensor_msgs::CameraInfoConstPtr& msg) {

  ROS_INFO_ONCE("camera_info received");

  cam_model_.fromCameraInfo(msg);
  
  Eigen::Matrix3d cameraMatrix;
  //cv::cv2eigen(cam_model_.intrinsicMatrix(), cameraMatrix); // TODO fix this
  for(int i = 0; i < 3; i++)
  for(int j = 0; j < 3; j++)
  cameraMatrix(i,j) = cam_model_.intrinsicMatrix()(i,j);
  
  Eigen::VectorXd distCoeffs(8);
  //cv::cv2eigen(cam_model_.distortionCoeffs(), distCoeffs); // TODO fix this
  distCoeffs << cam_model_.distortionCoeffs()(0),
                cam_model_.distortionCoeffs()(1),
                cam_model_.distortionCoeffs()(2),
                cam_model_.distortionCoeffs()(3),
                cam_model_.distortionCoeffs()(4),
                cam_model_.distortionCoeffs()(5),
                cam_model_.distortionCoeffs()(6),
                cam_model_.distortionCoeffs()(7);
  
  detector_->model.setCameraProperties(cameraMatrix, distCoeffs);
  
  cam_info_sub_.shutdown();
  
  if (continuous_mode_) cam_image_sub_ = it_.subscribe("/cam_image_topic", 1, &umfLocalizerNode::cameraImageCallback, this);
  
}

void umfLocalizerNode::trCallback(const ros::TimerEvent& event) {

    tr_.stamp_ = ros::Time::now();
    br_.sendTransform(tr_);

}

geometry_msgs::Pose umfLocalizerNode::inverse(geometry_msgs::Pose pose) {
  Eigen::Affine3d pose_eigen;
  tf::poseMsgToEigen(pose, pose_eigen);
  tf::poseEigenToMsg(pose_eigen.inverse(), pose);
  return pose;
}

void umfLocalizerNode::cameraImageCallback(const sensor_msgs::ImageConstPtr& msg) {

  ROS_INFO_ONCE("camera image received");
  
  cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::MONO8);
  IplImage iplimg = cv_ptr->image;

  ImageGray *imgGray = new ImageGray(iplimg.width, iplimg.height, false, iplimg.widthStep);
  imgGray->data = iplimg.imageData;
  
  bool success = false;

  if (!continuous_mode_ && (ros::Time::now() - localization_start_) > localization_to_) {

      ROS_INFO("timeout");
      art_umf_localizer::LocalizeAgainstUMFResult res;
      res.result = res.RES_DETECTION_FAILED;
      as_.setAborted(res);
      cam_image_sub_.shutdown();
      return;

  }
  
  try{
  
      success = detector_->update(imgGray, -1.f);
      
  } catch(DetectionTimeoutException &)
  {
      ROS_WARN("Detection timed out.");
  }
  
  if (success)
  {
    
    double cameraPos[3];
    double cameraQuat[4];

    detector_->model.getCameraPosRot(cameraPos, cameraQuat);
    
    geometry_msgs::PoseStamped pose;
    pose.header.frame_id = msg->header.frame_id;
    pose.header.stamp = msg->header.stamp;
    pose.pose.position.x = square_size_ * cameraPos[0];
    pose.pose.position.y = square_size_ * cameraPos[1];
    pose.pose.position.z = square_size_ * cameraPos[2];
    pose.pose.orientation.x = cameraQuat[1];
    pose.pose.orientation.y = cameraQuat[2];
    pose.pose.orientation.z = cameraQuat[3];
    pose.pose.orientation.w = cameraQuat[0];

    pose.pose = inverse(pose.pose);

    if (!tfl_.waitForTransform(robot_frame_, msg->header.frame_id, msg->header.stamp, ros::Duration(0.2))) {

      ROS_WARN_STREAM_THROTTLE(2.0, "Transform from " << msg->header.frame_id << " to " << robot_frame_ << " not available!");
      return;

    }

    try {

        tfl_.transformPose(robot_frame_, pose, pose);

    } catch (tf::TransformException ex) {

        ROS_ERROR("%s",ex.what());
        return;
      }

    pose_pub_.publish(pose);

    tf::Transform tr;
    tr.setOrigin(tf::Vector3(pose.pose.position.x, pose.pose.position.y, pose.pose.position.z));
    tr.setRotation(tf::Quaternion(pose.pose.orientation.x, pose.pose.orientation.y, pose.pose.orientation.z, pose.pose.orientation.w));

    // TODO do some filtering - from more detections
    tr_ = tf::StampedTransform(tr.inverse(), msg->header.stamp, world_frame_, robot_frame_);

    br_.sendTransform(tr_);

    if (!continuous_mode_) {

        ROS_INFO("finished");
        art_umf_localizer::LocalizeAgainstUMFResult res;
        res.result = res.RES_FINISHED;
        as_.setSucceeded(res);
        cam_image_sub_.shutdown();

        tr_timer_ = nh_.createTimer(ros::Duration(0.1), &umfLocalizerNode::trCallback, this);

    }

  } else {
  
      ROS_WARN_THROTTLE(2.0, "Detection failed.");

  }

}

umfLocalizerNode::~umfLocalizerNode() {

}

int main(int argc, char* argv[])
{

    ros::init(argc, argv, "umf_localizer_node");
    ros::NodeHandle n("~");
    
    umfLocalizerNode node(n);
    if (!node.init()) return -1;
    ros::spin();
    
    return 0;
}
