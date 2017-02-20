// Copyright 2016 Robo@FIT

#include <string>
#include <ros/ros.h>
#include "art_pr2_grasping/pr2grasp.h"
#include <actionlib/server/simple_action_server.h>
#include <art_msgs/PickPlaceAction.h>
#include <tf/transform_listener.h>

#ifndef ART_PR2_GRASPING_ACTION_SERVER_H
#define ART_PR2_GRASPING_ACTION_SERVER_H

namespace art_pr2_grasping
{

class artActionServer : public artPr2Grasping
{
public:
    artActionServer(boost::shared_ptr<tf::TransformListener> tfl, boost::shared_ptr<Objects> objects, std::string group_name, std::string default_target, std::string gripper_state_topic) : private_nh_("~"), max_attempts_(3), artPr2Grasping(tfl, objects, group_name, default_target, gripper_state_topic)
    {

        as_.reset(new actionlib::SimpleActionServer<art_msgs::PickPlaceAction>(
                      nh_, "/art/pr2/" + group_name_ + "/pp", boost::bind(&artActionServer::executeCB, this, _1), false));

    }

    bool init()
    {
        // todo - tohle poradi (ready, addtable) je dobre pro simulaci - v realu by
        // to bylo lepsi naopak??
        if (!getReady())
            return false;

        ros::Duration(1).sleep();

        if (getGripperValue() == 1000) { // TODO exception / NaN

            ROS_ERROR_NAMED(group_name_, "Can't get gripper state");
            return false;
        }

        as_->start();
        return true;
    }

private:

    boost::shared_ptr<actionlib::SimpleActionServer<art_msgs::PickPlaceAction> > as_;
    int max_attempts_;

    void executeCB(const art_msgs::PickPlaceGoalConstPtr &goal)
    {
        art_msgs::PickPlaceResult res;
        art_msgs::PickPlaceFeedback f;

        ROS_INFO_NAMED(group_name_, "Got goal, operation: %d", goal->operation);

        // TODO check /art/pr2/xyz_arm/interaction/state topic?

        switch(goal->operation) {

        case art_msgs::PickPlaceGoal::GET_READY: {

            if (!getReady()) {

                res.result = art_msgs::PickPlaceResult::FAILED;
                as_->setAborted(res, "failed to get ready");
                return;
            }

            res.result = art_msgs::PickPlaceResult::SUCCESS;
            as_->setSucceeded(res);


        } break;

        case art_msgs::PickPlaceGoal::PICK_OBJECT_ID: {

            if (!objects_.isKnownObject(goal->object))
            {
                res.result = art_msgs::PickPlaceResult::BAD_REQUEST;
                as_->setAborted(res, "unknown object id");
                return;
            }

            int tries = max_attempts_;

            f.operation = art_msgs::PickPlaceGoal::PICK;

            bool grasped = false;

            while (tries > 0 && !as_->isPreemptRequested())
            {
                f.attempt = (max_attempts_ - tries) + 1;
                ROS_INFO("Pick %d", f.attempt);
                tries--;
                as_->publishFeedback(f);

                grasped = pick(goal->object);  // todo flag if it make sense to try again
                // (type of failure)
                if (grasped)
                    break;
            }

            if (as_->isPreemptRequested())
            {
                as_->setPreempted(res, "pick cancelled");
                return;
            }

            if (!grasped)
            {
                res.result = art_msgs::PickPlaceResult::FAILURE;
                as_->setAborted(res, "pick failed");
                return;
            }

            res.result = art_msgs::PickPlaceResult::SUCCESS;
            as_->setSucceeded(res);

        } break;

        case art_msgs::PickPlaceGoal::PICK_FROM_FEEDER: {

            res.result = art_msgs::PickPlaceResult::BAD_REQUEST;
            as_->setAborted(res, "PICK_FROM_FEEDER is not implemented yet");
            return;

        } break;

        case art_msgs::PickPlaceGoal::PLACE_TO_POSE: {

            geometry_msgs::PoseStamped p2 = goal->place_pose;  // place goal

            // do transformation in advance - before timestamp get too old
            if (!transformPose(p2))
            {
                res.result = art_msgs::PickPlaceResult::FAILURE;
                as_->setAborted(res, "failed to transform pose");
                return;
            }

            if (!hasGraspedObject())
            {
                ROS_ERROR_NAMED(group_name_, "No object grasped - can't do PLACE.");
                res.result = art_msgs::PickPlaceResult::FAILURE;
                as_->setAborted(res, "No object grasped!");
                return;
            }

            int tries = max_attempts_;

            f.operation = goal->operation == art_msgs::PickPlaceGoal::PLACE;

            bool placed = false;

            while (tries > 0 && !as_->isPreemptRequested())
            {
                f.attempt = (max_attempts_ - tries) + 1;
                ROS_INFO_NAMED(group_name_, "Place %d", f.attempt);
                tries--;
                as_->publishFeedback(f);
                placed = place(p2.pose, goal->z_axis_angle_increment, goal->keep_orientation);
                if (placed)
                    break;
            }

            if (as_->isPreemptRequested())
            {
                as_->setPreempted(res, "place cancelled");
                return;
            }

            if (!placed)
            {
                ROS_ERROR_NAMED(group_name_, "Unable to place the object.");
                res.result = art_msgs::PickPlaceResult::FAILURE;
                as_->setAborted(res, "place failed");
                return;
            }

            res.result = art_msgs::PickPlaceResult::SUCCESS;
            as_->setSucceeded(res);

        } break;

        default:

            res.result = art_msgs::PickPlaceResult::BAD_REQUEST;
            as_->setAborted(res, "Unknown operation");
        }

    }
};
}  // namespace art_pr2_grasping

#endif  // ART_PR2_GRASPING_ACTION_SERVER_H
