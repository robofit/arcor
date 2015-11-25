#include "umf_localizer_node.h"
#include <opencv2/core/eigen.hpp>

using namespace umf_localizer_node;
using namespace umf;
using namespace std;

umfLocalizerNode::umfLocalizerNode(ros::NodeHandle& nh): it_(nh) {

  nh_ = nh;
  detector_.reset(new umf::UMFDetector<1>(UMF_FLAG_ITER_REFINE|UMF_FLAG_TRACK_POS| UMF_FLAG_SUBWINDOWS | UMF_FLAG_SUBPIXEL));
  detector_->setTrackingFlags(UMF_TRACK_MARKER | UMF_TRACK_SCANLINES);
  
  cam_info_sub_ = nh_.subscribe("cam_info_topic", 10, &umfLocalizerNode::cameraInfoCallback, this);

}

void umfLocalizerNode::cameraInfoCallback(const sensor_msgs::CameraInfoConstPtr& msg) {

  ROS_INFO_ONCE("camera_info received");

  cam_model_.fromCameraInfo(msg);
  
  Eigen::Matrix3d cameraMatrix;
  cv::cv2eigen(cam_model_.intrinsicMatrix(), cameraMatrix);
  
  Eigen::VectorXd distCoeffs(8);
  cv::cv2eigen(cam_model_.distortionCoeffs(), distCoeffs);
  
  detector_->model.setCameraProperties(cameraMatrix, distCoeffs);
  
  cam_info_sub_.shutdown();
  
  cam_image_sub_ = it_.subscribe("cam_image_topic", 10, &umfLocalizerNode::cameraImageCallback, this);
  
}

void umfLocalizerNode::cameraImageCallback(const sensor_msgs::ImageConstPtr& msg) {

  ROS_INFO_ONCE("camera image received");

}


umfLocalizerNode::~umfLocalizerNode() {

}

int main(int argc, char* argv[])
{

    ros::init(argc, argv, "umf_localizer_node");
    ros::NodeHandle n;
    
    umfLocalizerNode node(n);
    ros::spin();
    
    return 0;
}
