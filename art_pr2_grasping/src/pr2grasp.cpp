// Copyright 2016 Robo@FIT

#include "art_pr2_grasping/action_server.h"

int main(int argc, char **argv)
{
  ros::init(argc, argv, "pr2_pick_place");

  ros::AsyncSpinner spinner(1);
  spinner.start();

  ROS_INFO("Starting...");

  art_pr2_grasping::artActionServer as;
  ros::Duration(1).sleep();
  if (!as.init())
  {
    ROS_ERROR("Init failed.");
    return 1;
  }

  ROS_INFO("Started - waiting for goals.");

  ros::waitForShutdown();
  return 0;
}
