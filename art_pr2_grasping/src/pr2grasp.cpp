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
  ps.pose.position.x = 0.6;
  ps.pose.position.y = 0.3;
  ps.pose.position.z = 0.8;
  ps.pose.orientation.x = 0.0;
  ps.pose.orientation.y = 0.0;
  ps.pose.orientation.z = 0.0;
  ps.pose.orientation.w = 1.0;

  gr.getReady();

  if (!gr.addTable(0.75, 0, 0, 1.5, 0.75, 0.75, "table1")) ROS_ERROR("failed to add table");
  ros::Duration(1).sleep();

  ROS_INFO("ready");

  float dx, dy;

  while(ros::ok()) {

      dx = static_cast <float> (rand()) / (static_cast <float> (RAND_MAX/0.2)) - 0.1;
      dy = static_cast <float> (rand()) / (static_cast <float> (RAND_MAX/0.2)) - 0.1;

      ps.pose.position.x = 0.6 + dx;
      ps.pose.position.y = 0.3 + dy;

      if (gr.pick("1", gr.LEFT, ps)) {

          dy = static_cast <float> (rand()) / (static_cast <float> (RAND_MAX/0.2)) - 0.1;
          ps.pose.position.y = 0.2 + dy;

          if (!gr.place("1", gr.LEFT, ps)) ROS_ERROR("left place failed");

      } else ROS_ERROR("left pick failed");

      gr.getReady(gr.LEFT);


      dx = static_cast <float> (rand()) / (static_cast <float> (RAND_MAX/0.2)) - 0.1;
      dy = static_cast <float> (rand()) / (static_cast <float> (RAND_MAX/0.2)) - 0.1;

      ps.pose.position.x = 0.6 + dx;
      ps.pose.position.y = -0.3 + dy;

      if (gr.pick("2", gr.RIGHT, ps)) {

          dy = static_cast <float> (rand()) / (static_cast <float> (RAND_MAX/0.2)) - 0.1;

          ps.pose.position.y = -0.2 + dy;

          if (!gr.place("2", gr.RIGHT, ps)) ROS_ERROR("right place failed");

      } else ROS_ERROR("right pick failed");

      gr.getReady(gr.RIGHT);

  }


  return 0;
}
