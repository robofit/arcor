#include "art_pr2_grasping/pr2grasp.h"
#include <actionlib/server/simple_action_server.h>
#include <art_pr2_grasping/pickplaceAction.h>
#include <tf/transform_listener.h>

#ifndef ART_PR2_GRASPING_ACTION_SERVER_H
#define ART_PR2_GRASPING_ACTION_SERVER_H

namespace art_pr2_grasping
{

class artActionServer
{

public:
  artActionServer(): nh_("~"),
    as_(nh_, "pp", boost::bind(&artActionServer::executeCB, this, _1), false),
    max_attempts_(3)
  {

  }

  bool init()
  {

    // todo - tohle poradi (ready, addtable) je dobre pro simulaci - v realu by to bylo lepsi naopak??
    if (!gr_.getReady()) return false;

    ros::Duration(1).sleep();

    // todo - read from param?
    if (!gr_.addTable(0.75, 0, 0, 1.5, 0.74, 0.80, "table1"))
    {

      ROS_ERROR("failed to add table");
      return false;
    }

    as_.start();
    return true;
  }

private:
  ros::NodeHandle nh_;
  artPr2Grasping gr_;
  actionlib::SimpleActionServer<pickplaceAction> as_;
  int max_attempts_;
  tf::TransformListener tfl_;

  bool transformPose(const artPr2Grasping::planning_group &g, geometry_msgs::PoseStamped &ps)
  {

    try
    {

      if (tfl_.waitForTransform(gr_.getPlanningFrame(g), ps.header.frame_id, ps.header.stamp, ros::Duration(5)))
      {

        tfl_.transformPose(gr_.getPlanningFrame(g), ps, ps);

      }
      else
      {

        ROS_ERROR_STREAM("Transform between" << gr_.getPlanningFrame(g) << "and " << ps.header.frame_id << " not available!");
        return false;
      }

    }
    catch (tf::TransformException ex)
    {

      ROS_ERROR("%s", ex.what());
      return false;
    }

    return true;
  }

  void executeCB(const pickplaceGoalConstPtr &goal)
  {

    pickplaceResult res;
    pickplaceFeedback f;
    artPr2Grasping::planning_group g;

    if (goal->arm == pickplaceGoal::LEFT_ARM) g = artPr2Grasping::LEFT;
    else if (goal->arm == pickplaceGoal::RIGHT_ARM) g = artPr2Grasping::RIGHT;
    else
    {

      res.result = pickplaceResult::BAD_REQUEST;
      as_.setAborted(res, "unknown planning group");
      return;
    }

    if (goal->operation != pickplaceGoal::PICK_AND_PLACE && goal->operation != pickplaceGoal::PICK && goal->operation != pickplaceGoal::PLACE)
    {

      res.result = pickplaceResult::BAD_REQUEST;
      as_.setAborted(res, "unknown operation");
      return;
    }

    ROS_INFO("Got goal for group: %d, operation: %d", goal->arm, goal->operation);

    geometry_msgs::PoseStamped p1 = goal->pose; // pick goal
    geometry_msgs::PoseStamped p2 = goal->pose2; // place goal

    // do all transformations in advance - before timestamps get too old
    if (goal->operation == pickplaceGoal::PICK_AND_PLACE || goal->operation == pickplaceGoal::PICK)
    {

      if (!transformPose(g, p1))
      {

        res.result = pickplaceResult::FAILURE;
        as_.setAborted(res, "failed to transform pick pose");
        return;
      }
    }

    if (goal->operation == pickplaceGoal::PICK_AND_PLACE || goal->operation == pickplaceGoal::PLACE)
    {

      if (!transformPose(g, p2))
      {

        res.result = pickplaceResult::FAILURE;
        as_.setAborted(res, "failed to transform pose");
        return;
      }
    }

    if (goal->operation == pickplaceGoal::PICK_AND_PLACE || goal->operation == pickplaceGoal::PICK)
    {

      if (goal->bb.dimensions.size() != 3)
      {

        res.result = pickplaceResult::BAD_REQUEST;
        as_.setAborted(res, "for pick/p&p a bounding box must be given ");
        return;
      }

      int tries = max_attempts_;

      f.operation = pickplaceGoal::PICK;

      bool grasped = false;

      while (tries > 0 && !as_.isPreemptRequested())
      {

        f.attempt = (max_attempts_ - tries) + 1;
        ROS_INFO("Pick %d", f.attempt);
        tries--;
        as_.publishFeedback(f);

        grasped = gr_.pick(goal->id, g, p1.pose, goal->bb); // todo flag if it make sense to try again (type of failure)
        if (grasped) break;
      }

      if (as_.isPreemptRequested())
      {

        gr_.getReady(g);
        as_.setPreempted(res, "pick cancelled");
        return;
      }

      if (!grasped)
      {

        ROS_ERROR("Unable to pick the object.");
        gr_.getReady(g);
        res.result = pickplaceResult::FAILURE;
        as_.setAborted(res, "pick failed");
        return;
      }

      ROS_INFO("Picked the object.");

      if (goal->operation == pickplaceGoal::PICK) gr_.getReady(g); // get ready and wait with object grasped

    }

    if (goal->operation == pickplaceGoal::PICK_AND_PLACE || goal->operation == pickplaceGoal::PLACE)
    {

      if (!gr_.hasGraspedObject(g))
      {

        ROS_ERROR("No object grasped - can't do PLACE.");
        res.result = pickplaceResult::FAILURE;
        as_.setAborted(res, "No object grasped!");
        return;
      }

      int tries = max_attempts_;

      f.operation = goal->operation == pickplaceGoal::PLACE;

      bool placed = false;

      while (tries > 0 && !as_.isPreemptRequested())
      {

        f.attempt = (max_attempts_ - tries) + 1;
        ROS_INFO("Place %d", f.attempt);
        tries--;
        as_.publishFeedback(f);
        placed = gr_.place(g, p2.pose, goal->free_z_axis);
        if (placed) break;
      }

      if (as_.isPreemptRequested())
      {

        gr_.getReady(g);
        as_.setPreempted(res, "place cancelled");
        return;
      }

      if (!placed)
      {

        ROS_ERROR("Unable to place the object.");
        gr_.getReady(g);
        res.result = pickplaceResult::FAILURE;
        as_.setAborted(res, "place failed");
        return;
      }

      ROS_INFO("Placed the object.");

      gr_.getReady(g);
    }

    res.result = pickplaceResult::SUCCESS;
    as_.setSucceeded(res);

  }

};
} // namespace art_pr2_grasping

#endif  // ART_PR2_GRASPING_ACTION_SERVER_H
