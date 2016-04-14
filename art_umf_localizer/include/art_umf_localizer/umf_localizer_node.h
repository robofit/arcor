#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <image_geometry/pinhole_camera_model.h>
#include <tf/tf.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <actionlib/server/simple_action_server.h>
#include <art_msgs/LocalizeAgainstUMFAction.h>
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

      geometry_msgs::Pose inverse(geometry_msgs::Pose pose);
      
      image_geometry::PinholeCameraModel cam_model_;
      
      ros::Subscriber cam_info_sub_;
      image_transport::ImageTransport it_;
      image_transport::Subscriber cam_image_sub_;
      
      tf::TransformBroadcaster br_;
      tf::TransformListener tfl_;
      
      ros::Publisher pose_pub_;
      
      std::string robot_frame_;
      std::string world_frame_;
      double square_size_;

      bool continuous_mode_;
      ros::Duration localization_to_;
      ros::Time localization_start_;

      tf::StampedTransform tr_;
      ros::Timer tr_timer_;
      void trCallback(const ros::TimerEvent& event);

      actionlib::SimpleActionServer<art_msgs::LocalizeAgainstUMFAction> as_;
      void goalCB();
      void preemptCB();

      int detection_cnt_;
      geometry_msgs::PoseStamped pose_filtered_;

      float filt_coef_pos_;
      float filt_coef_rot_;

  };

}

#endif
