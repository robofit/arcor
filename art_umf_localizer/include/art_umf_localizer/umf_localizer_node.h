#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <image_geometry/pinhole_camera_model.h>
#include <tf/tf.h>
#include "umf.h"


#ifndef UMF_LOC_NODE_H_
#define UMF_LOC_NODE_H_

namespace umf_localizer_node {

  class umfLocalizerNode {
  
    public:
    
      umfLocalizerNode(ros::NodeHandle& nh);
      ~umfLocalizerNode();
      
    private:
    
      ros::NodeHandle nh_;
    
      std::shared_ptr<umf::UMFDetector<1>> detector_;
      
      void cameraInfoCallback(const sensor_msgs::CameraInfoConstPtr& msg);
      void cameraImageCallback(const sensor_msgs::ImageConstPtr& msg);
      
      image_geometry::PinholeCameraModel cam_model_;
      
      ros::Subscriber cam_info_sub_;
      image_transport::ImageTransport it_;
      image_transport::Subscriber cam_image_sub_;
  
  };

}

#endif
