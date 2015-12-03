#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <image_geometry/pinhole_camera_model.h>
#include <tf/tf.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include "umf.h"


#ifndef UMF_LOC_NODE_H_
#define UMF_LOC_NODE_H_

namespace umf_localizer_node {

  class umfLocalizerNode {
  
    public:
    
      umfLocalizerNode(ros::NodeHandle& nh);
      ~umfLocalizerNode();
      
      bool init();
      
    private:
    
      ros::NodeHandle nh_;
    
      std::shared_ptr<umf::UMFDetector<1>> detector_;
      
      void cameraInfoCallback(const sensor_msgs::CameraInfoConstPtr& msg);
      void cameraImageCallback(const sensor_msgs::ImageConstPtr& msg);
      
      image_geometry::PinholeCameraModel cam_model_;
      
      ros::Subscriber cam_info_sub_;
      image_transport::ImageTransport it_;
      image_transport::Subscriber cam_image_sub_;
      
      tf::TransformBroadcaster br_;
      tf::TransformListener tfl_;
      
      ros::Publisher pose_pub_;
      
      std::string robot_frame_;
      std::string world_frame_;
  
  };

}

#endif
