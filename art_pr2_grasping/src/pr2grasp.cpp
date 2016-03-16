#include "art_pr2_grasping/pr2grasp.h"

using namespace art_pr2_grasping;

int main(int argc, char **argv)
{
  ros::init (argc, argv, "pr2_pick_place");

  ros::AsyncSpinner spinner(1);
  spinner.start();

  artPr2Grasping gr;

  ros::Duration(1).sleep();

  geometry_msgs::PoseStamped ps;

  ps.header.frame_id = "base_footprint";
  ps.pose.position.x = 0.75;
  ps.pose.position.y = 0.3;
  ps.pose.position.z = 0.75;
  ps.pose.orientation.x = 0.0;
  ps.pose.orientation.y = 0.0;
  ps.pose.orientation.z = 0.0;
  ps.pose.orientation.w = 1.0;

  gr.getReady();

  ROS_INFO("ready");

  if (!gr.pick("1", "left_arm", ps)) {

      ROS_ERROR("pick failed");
      ros::spin();
  }

  ps.pose.position.y = 0.1;

  if (!gr.place("1", "left_arm", ps)) {

      ROS_ERROR("place failed");
  }

  ros::spin();

  return 0;
}
