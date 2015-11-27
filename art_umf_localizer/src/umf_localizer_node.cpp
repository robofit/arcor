#include "umf_localizer_node.h"
#include <opencv2/core/eigen.hpp>
#include <sensor_msgs/image_encodings.h>
#include <cv_bridge/cv_bridge.h>

using namespace umf_localizer_node;
using namespace umf;
using namespace std;

umfLocalizerNode::umfLocalizerNode(ros::NodeHandle& nh): it_(nh) {

  nh_ = nh;
  detector_.reset(new umf::UMFDetector<1>(UMF_FLAG_ITER_REFINE|UMF_FLAG_TRACK_POS| UMF_FLAG_SUBWINDOWS | UMF_FLAG_SUBPIXEL));
  detector_->setTrackingFlags(UMF_TRACK_MARKER | UMF_TRACK_SCANLINES);
  
  cam_info_sub_ = nh_.subscribe("/cam_info_topic", 1, &umfLocalizerNode::cameraInfoCallback, this);
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
  
  ROS_INFO("Ready");
  return true;

} 

void umfLocalizerNode::cameraInfoCallback(const sensor_msgs::CameraInfoConstPtr& msg) {

  ROS_INFO_ONCE("camera_info received");

  cam_model_.fromCameraInfo(msg);
  
  Eigen::Matrix3d cameraMatrix;
  //cv::cv2eigen(cam_model_.intrinsicMatrix(), cameraMatrix);
  for(int i = 0; i < 3; i++)
  for(int j = 0; j < 3; j++)
  cameraMatrix(i,j) = cam_model_.intrinsicMatrix()(i,j);
  
  Eigen::VectorXd distCoeffs(8);
  //cv::cv2eigen(cam_model_.distortionCoeffs(), distCoeffs);
  distCoeffs << 0, 0, 0, 0, 0, 0, 0, 0;
  
  detector_->model.setCameraProperties(cameraMatrix, distCoeffs);
  
  cam_info_sub_.shutdown();
  
  cam_image_sub_ = it_.subscribe("/cam_image_topic", 1, &umfLocalizerNode::cameraImageCallback, this);
  
}

void umfLocalizerNode::cameraImageCallback(const sensor_msgs::ImageConstPtr& msg) {

  ROS_INFO_ONCE("camera image received");
  
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
    Eigen::Vector3d angles;

    detector_->model.getWorldPosRot(cameraPos, rotationQuat);
     
    //std::cout << cameraPos[0] << " " << cameraPos[1] << " " << cameraPos[2] << std::endl;
     
    tf::Transform transform;
    transform.setOrigin( tf::Vector3(cameraPos[0], cameraPos[1], cameraPos[2]) );
    tf::Quaternion q(rotationQuat[0], rotationQuat[1], rotationQuat[2], rotationQuat[3]);
    transform.setRotation(q);
    br_.sendTransform(tf::StampedTransform(transform, msg->header.stamp, "marker", msg->header.frame_id));
     
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
