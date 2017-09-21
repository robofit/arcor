// Copyright 2016 Robo@FIT

#ifndef ART_PR2_GRASPING_ACTION_SERVER_H
#define ART_PR2_GRASPING_ACTION_SERVER_H

#include <string>
#include <ros/ros.h>
#include "art_pr2_grasping/pr2grasp.h"
#include "art_pr2_grasping/objects.h"
#include <actionlib/server/simple_action_server.h>
#include <art_msgs/PickPlaceAction.h>
#include <tf/transform_listener.h>

namespace art_pr2_grasping
{
class artActionServer : public artPr2Grasping
{
public:
  artActionServer(boost::shared_ptr<tf::TransformListener> tfl, boost::shared_ptr<Objects> objects,
                  std::string group_name, std::string default_target, std::string gripper_state_topic);

  bool init();

private:
  boost::shared_ptr<actionlib::SimpleActionServer<art_msgs::PickPlaceAction> > as_;
  int max_attempts_;

  void executeCB(const art_msgs::PickPlaceGoalConstPtr& goal);
};
}  // namespace art_pr2_grasping

#endif  // ART_PR2_GRASPING_ACTION_SERVER_H
