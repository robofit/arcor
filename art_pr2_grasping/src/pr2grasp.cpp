#include "art_pr2_grasping/pr2grasp.h"

using namespace art_pr2_grasping;

int main(int argc, char **argv)
{
  ros::init (argc, argv, "pr2_pick_place");

  ros::AsyncSpinner spinner(1);
  spinner.start();

  artActionServer as;
  ros::Duration(1).sleep();
  if (!as.init()) return 0;
  ros::spin();

  return 0;
}
