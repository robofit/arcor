// Copyright 2016 Robo@FIT

#include <string>
#include "art_pr2_grasping/pr2grasp.h"
#include <actionlib/server/simple_action_server.h>
#include <art_msgs/PickPlaceAction.h>
#include <tf/transform_listener.h>
#include <std_msgs/String.h>
#include <pr2_controllers_msgs/JointControllerState.h>

#ifndef ART_PR2_GRASPING_ACTION_SERVER_H
#define ART_PR2_GRASPING_ACTION_SERVER_H

namespace art_pr2_grasping
{
class artActionServer
{
public:
    artActionServer() : private_nh_("~"), max_attempts_(3)
    {
        tfl_.reset(new tf::TransformListener());
        private_nh_.param<std::string>("group_name", group_name_, "left_arm");
        as_.reset(new actionlib::SimpleActionServer<art_msgs::PickPlaceAction>(
                      nh_, "/art/pr2/" + group_name_ + "/pp", boost::bind(&artActionServer::executeCB, this, _1), false));
        grasped_object_ = nh_.advertise<std_msgs::String>("/art/pr2/" + group_name_ + "/grasped_object", 1, true);
        private_nh_.param<std::string>("gripper_state", gripper_state_topic_, "/l_gripper_controller/state");

    }

    bool init()
    {
        // todo - tohle poradi (ready, addtable) je dobre pro simulaci - v realu by
        // to bylo lepsi naopak??
        if (!gr_.getReady())
            return false;

        ros::Duration(1).sleep();

        // todo - read from param?
        if (!gr_.addTable(0.75, 0, 0, 1.5, 0.78, 0.70, "table1"))
        {
            ROS_ERROR("failed to add table");
            return false;
        }

        publish_object("");

        pr2_controllers_msgs::JointControllerStateConstPtr msg =
                ros::topic::waitForMessage<pr2_controllers_msgs::JointControllerState>(gripper_state_topic_, ros::Duration(1));

        if (!msg)
        {
            ROS_ERROR("Can't get gripper state");
            return false;
        }

        as_->start();
        return true;
    }

private:
    ros::NodeHandle private_nh_;
    ros::NodeHandle nh_;
    artPr2Grasping gr_;
    boost::shared_ptr<actionlib::SimpleActionServer<art_msgs::PickPlaceAction> > as_;
    int max_attempts_;
    boost::shared_ptr<tf::TransformListener> tfl_;

    ros::Publisher grasped_object_;

    std::string group_name_;
    std::string gripper_state_topic_;

    void publish_object(const std::string &obj)
    {
        std_msgs::String str;
        str.data = obj;
        grasped_object_.publish(str);
    }

    void executeCB(const art_msgs::PickPlaceGoalConstPtr &goal)
    {
        art_msgs::PickPlaceResult res;
        art_msgs::PickPlaceFeedback f;

        ROS_INFO("Got goal, operation: %d", goal->operation);

        // TODO check /art/pr2/xyz_arm/interaction/state topic!

        switch(goal->operation) {

        case art_msgs::PickPlaceGoal::GET_READY: {

            if (!gr_.getReady()) {

                res.result = art_msgs::PickPlaceResult::FAILED;
                as_->setAborted(res, "failed to get ready");
                return;
            }

            res.result = art_msgs::PickPlaceResult::SUCCESS;
            as_->setSucceeded(res);


        } break;

        case art_msgs::PickPlaceGoal::PICK_OBJECT_ID: {

            if (!gr_.isKnownObject(goal->id))
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

                grasped = gr_.pick(goal->id);  // todo flag if it make sense to try again
                // (type of failure)
                if (grasped)
                    break;
            }

            if (as_->isPreemptRequested())
            {
                gr_.getReady();
                as_->setPreempted(res, "pick cancelled");
                return;
            }

            if (!grasped)
            {
                ROS_ERROR("Unable to pick the object.");
                gr_.getReady();
                res.result = art_msgs::PickPlaceResult::FAILURE;
                as_->setAborted(res, "pick failed");
                return;
            }

            pr2_controllers_msgs::JointControllerStateConstPtr msg =
                    ros::topic::waitForMessage<pr2_controllers_msgs::JointControllerState>(gripper_state_topic_);
            if (msg->process_value < 0.005)
            {
                gr_.resetGraspedObject();
                gr_.getReady();
                ROS_ERROR("Gripper is closed - object missed or dropped :-(");
                res.result = art_msgs::PickPlaceResult::FAILURE;
                as_->setAborted(res, "gripper closed after pick");
                return;
            }

            ROS_INFO("Picked the object.");
            publish_object(goal->id);

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
            if (!gr_.transformPose(p2))
            {
                res.result = art_msgs::PickPlaceResult::FAILURE;
                as_->setAborted(res, "failed to transform pose");
                return;
            }

            if (!gr_.hasGraspedObject())
            {
                ROS_ERROR("No object grasped - can't do PLACE.");
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
                ROS_INFO("Place %d", f.attempt);
                tries--;
                as_->publishFeedback(f);
                placed = gr_.place(p2.pose, goal->z_axis_angle_increment, goal->keep_orientation);
                if (placed)
                    break;
            }

            if (as_->isPreemptRequested())
            {
                gr_.getReady();
                as_->setPreempted(res, "place cancelled");
                return;
            }

            if (!placed)
            {
                ROS_ERROR("Unable to place the object.");
                gr_.getReady();
                res.result = art_msgs::PickPlaceResult::FAILURE;
                as_->setAborted(res, "place failed");
                return;
            }

            ROS_INFO("Placed the object.");
            publish_object("");

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
