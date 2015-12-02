#include "umf_localizer_node.h"
#include <opencv2/core/eigen.hpp>
#include <sensor_msgs/image_encodings.h>
#include <cv_bridge/cv_bridge.h>

using namespace umf_localizer_node;
using namespace umf;
using namespace std;

umfLocalizerNode::umfLocalizerNode(ros::NodeHandle& nh): it_(nh) {

  nh_ = nh;
  detector_.reset(new umf::UMFDetector<1>(UMF_FLAG_ITER_REFINE| /*UMF_FLAG_TRACK_POS|*/ UMF_FLAG_SUBWINDOWS | UMF_FLAG_SUBPIXEL));
  detector_->setTrackingFlags(UMF_TRACK_MARKER | UMF_TRACK_SCANLINES);
  
  cam_info_sub_ = nh_.subscribe("/cam_info_topic", 1, &umfLocalizerNode::cameraInfoCallback, this);
  pose_pub_ = nh_.advertise<geometry_msgs::PoseStamped>("pose", 10);
  
}

bool umfLocalizerNode::init() {

  string marker;

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
  
  ROS_INFO("Ready");
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
  
  cam_image_sub_ = it_.subscribe("/cam_image_topic", 1, &umfLocalizerNode::cameraImageCallback, this);
  
}

void umfLocalizerNode::cameraImageCallback(const sensor_msgs::ImageConstPtr& msg) {

  ROS_INFO_ONCE("camera image received");
  
  tf::StampedTransform cam_to_base;
  
  try {
  
    tfl_.lookupTransform(msg->header.frame_id, robot_frame_, msg->header.stamp, cam_to_base);
    
  }
  catch (tf::TransformException ex) {
  
    ROS_ERROR("%s",ex.what());
    return;
  }
  
  cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::MONO8);
  IplImage iplimg = cv_ptr->image;

  ImageGray *imgGray = new ImageGray(iplimg.width, iplimg.height, false, iplimg.widthStep);
  imgGray->data = iplimg.imageData;
  
  bool success = false;
  
  try{
  
      success = detector_->update(imgGray, -1.f);
      
  } catch(DetectionTimeoutException &)
  {
      ROS_WARN("Detection timed out.");
  }
  
  if (success)
  {

    double cameraPos[3];
    double rotationQuat[4];

    detector_->model.getWorldPosRot(cameraPos, rotationQuat);
     
    tf::Transform transform;
    transform.setOrigin( tf::Vector3(cameraPos[0], cameraPos[1], cameraPos[2]) );
    tf::Quaternion q(rotationQuat[0], rotationQuat[1], rotationQuat[2], rotationQuat[3]);
    transform.setRotation(q);
    
    transform = transform*cam_to_base;
    
    br_.sendTransform(tf::StampedTransform(transform, msg->header.stamp, "marker", robot_frame_));
    
    geometry_msgs::PoseStamped pose;
    pose.header.frame_id = "marker";
    pose.header.stamp = msg->header.stamp;
    pose.pose.position.x = transform.getOrigin().getX();
    pose.pose.position.y = transform.getOrigin().getY();
    pose.pose.position.z = transform.getOrigin().getZ();
    pose.pose.orientation.x = transform.getRotation().getX();
    pose.pose.orientation.y = transform.getRotation().getY();
    pose.pose.orientation.z = transform.getRotation().getZ();
    pose.pose.orientation.w = transform.getRotation().getW();
    
    pose_pub_.publish(pose);
     
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
