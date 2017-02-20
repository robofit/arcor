// Copyright 2016 Robo@FIT

#include <string>
#include "art_pr2_grasping/action_server.h"
#include "objects.h"
#include <tf/transform_listener.h>
#include <boost/thread/recursive_mutex.hpp>
#include <ros/ros.h>

#ifndef ART_PR2_GRASPING_NODE_H
#define ART_PR2_GRASPING_NODE_H

namespace art_pr2_grasping
{

class GraspingNode
{
public:
    GraspingNode() : private_nh_("~"), target_frame_("marker")
    {
        tfl_.reset(new tf::TransformListener());
        objects_.reset(new Objects(tfl_, target_frame_));

        std::vector<std::string> groups;
        std::vector<std::string> gripper_state_topics;
        std::vector<std::string> default_poses;

        private_nh_.getParam("groups", groups);
        private_nh_.getParam("gripper_state_topics", gripper_state_topics);
        private_nh_.getParam("default_poses", default_poses);

        ROS_ASSERT(groups.size() == gripper_state_topics.size() && groups.size() == default_poses.size() && groups.size() > 0);

        for (int i=0; i < groups.size(); i++) {

            action_servers_.push_back(artActionServer(tfl_, objects_, groups[i], default_poses[i], gripper_state_topics[i]));
        }

        // TODO visual_tools + callback from objects + publish/remove collision objects

    }

    bool init() {

        for (int i=0; i < action_servers_.size(); i++) {

            if (!action_servers_[i].init()) return false;
        }

        return true;
    }

private:

    std::vector<artActionServer> action_servers_;
    boost::shared_ptr<tf::TransformListener> tfl_;
    boost::shared_ptr<Objects> objects_;
    const std::string target_frame_;

};
}  // namespace art_pr2_grasping

#endif  // ART_PR2_GRASPING_NODE_H
