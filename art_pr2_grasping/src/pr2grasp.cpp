#include "art_pr2_grasping/pr2grasp.h"

using namespace art_pr2_grasping;

int main(int argc, char **argv)
{
  ros::init (argc, argv, "pr2_pick_place");

  artPr2Grasping gr();

  ros::spin();

  return 0;
}
