// Copyright 2016 Robo@FIT

#ifndef ART_PR2_GRASPING_NODE_H
#define ART_PR2_GRASPING_NODE_H

#include <string>
#include <vector>
#include "art_pr2_grasping/action_server.h"
#include "art_pr2_grasping/objects.h"
#include <tf/transform_listener.h>
#include <boost/thread/recursive_mutex.hpp>
#include <ros/ros.h>

namespace art_pr2_grasping
{
class GraspingNode
{
public:
  GraspingNode();
  bool init();

private:
  std::vector<boost::shared_ptr<artActionServer> > action_servers_;
  boost::shared_ptr<tf::TransformListener> tfl_;
  boost::shared_ptr<Objects> objects_;
  const std::string target_frame_;
  ros::NodeHandle nh_;
};
}  // namespace art_pr2_grasping

#endif  // ART_PR2_GRASPING_NODE_H
