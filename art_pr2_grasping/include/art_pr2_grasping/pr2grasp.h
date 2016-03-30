#include <string>
#include <vector>
#include "art_pr2_grasping/planning_group.h"
#include <pr2_controllers_msgs/PointHeadAction.h>
#include <actionlib/client/simple_action_client.h>

// adapted from https://github.com/davetcoleman/baxter_cpp/blob/hydro-devel/baxter_pick_place/src/block_pick_place.cpp

#ifndef ART_PR2_GRASPING_PR2GRASP_H
#define ART_PR2_GRASPING_PR2GRASP_H

namespace art_pr2_grasping
{

typedef actionlib::SimpleActionClient<pr2_controllers_msgs::PointHeadAction> PointHeadClient;

class artPr2Grasping
{

private:
  static const int PLANNING_GROUPS = 2;

  boost::scoped_ptr<artPlanningGroup> groups_[PLANNING_GROUPS];
  std::string group_name_[PLANNING_GROUPS];
  boost::scoped_ptr<PointHeadClient> head_;

  ros::NodeHandle nh_;

public:
  artPr2Grasping()
    : nh_("~")
  {

    group_name_[LEFT] = "left";
    group_name_[RIGHT] = "right";

    for (int i = 0; i < PLANNING_GROUPS; i++)
    {

      groups_[i].reset(new artPlanningGroup(group_name_[i]));
      groups_[i]->visual_tools_->setMuted(false);
      groups_[i]->visual_tools_->publishRemoveAllCollisionObjects();
      groups_[i]->grasped_object_.reset();
    }

    head_.reset(new PointHeadClient("/head_traj_controller/point_head_action", true));
    if (!head_->waitForServer(ros::Duration(5)))
    {

      ROS_WARN("Point head action not available!");
    }

  }

  enum planning_group {LEFT, RIGHT};

  //! Points the high-def camera frame at a point in a given frame
  void lookAt(geometry_msgs::Point pt)
  {

    pr2_controllers_msgs::PointHeadGoal goal;
    geometry_msgs::PointStamped point;
    point.header.frame_id = "base_footprint"; // todo - group/its planning frame as method param
    point.point = pt;
    goal.target = point;
    goal.pointing_frame = "high_def_frame"; // todo kinect?
    goal.pointing_axis.x = 1;
    goal.pointing_axis.y = 0;
    goal.pointing_axis.z = 0;
    goal.min_duration = ros::Duration(0.5);
    goal.max_velocity = 0.5;
    head_->sendGoal(goal);
    //head_->waitForResult(ros::Duration(2));
  }

  bool getReady(const planning_group & group)
  {

    geometry_msgs::PoseStamped ps;
    ps.header.frame_id = groups_[group]->move_group_->getPlanningFrame();
    ps.header.stamp = ros::Time::now();

    // todo specify as joint target?
    ps.pose.position.x = 0.093;
    ps.pose.position.y = 0.7;
    ps.pose.position.z = 1.0;
    ps.pose.orientation.x = -0.001;
    ps.pose.orientation.y = 0.320;
    ps.pose.orientation.z = -0.001;
    ps.pose.orientation.w = 0.947;

    if (group == RIGHT) ps.pose.position.y = -0.7;

    groups_[group]->move_group_->setPoseTarget(ps);

    if (!groups_[group]->move_group_->move())
    {

      ROS_WARN("Failed to get ready.");
      return false;

    }

    return true;
  }

  bool getReady()
  {

    bool ret = true;

    for (int i = 0; i < PLANNING_GROUPS; i++)
    {

      if (!getReady((planning_group)i)) ret = false;
    }

    return ret;
  }

  // todo moznost zadat jako PoseStamped?
  bool addTable(double x, double y, double angle, double width, double height, double depth, std::string name)
  {

    ROS_INFO("Adding table: %s", name.c_str());


    for (int i = 0; i < PLANNING_GROUPS; i++)
    {

      groups_[i]->visual_tools_->cleanupCO(name);

      for (int j = 0; j < 3; j++) // hmm, sometimes the table is not added
      {
        if (!groups_[i]->visual_tools_->publishCollisionTable(x, y, angle, width, height, depth, name)) return false;
        groups_[i]->move_group_->setSupportSurfaceName(name);
        ros::Duration(1).sleep();
      }
    }

    return true;
  }

  bool place(const planning_group &group, const geometry_msgs::Pose &ps, bool free_z_axis = false)
  {

    if (!groups_[group]->grasped_object_)
    {

      ROS_ERROR("No object to place.");
      return false;
    }

    lookAt(ps.position);

    std::vector<moveit_msgs::PlaceLocation> place_locations;

    geometry_msgs::PoseStamped pose_stamped;
    pose_stamped.header.frame_id = getPlanningFrame(group);
    pose_stamped.header.stamp = ros::Time::now();
    pose_stamped.pose = ps;

    groups_[group]->visual_tools_->publishBlock(ps, moveit_visual_tools::ORANGE, groups_[group]->grasped_object_->shape.dimensions[0], groups_[group]->grasped_object_->shape.dimensions[1], groups_[group]->grasped_object_->shape.dimensions[2]);

    double angle_increment = 0;

    if (free_z_axis) angle_increment = M_PI / 8;

    // Create 360 degrees of place location rotated around a center
    for (double angle = 0; angle < 2 * M_PI; angle += angle_increment)
    {
      pose_stamped.pose = ps;

      // Orientation
      Eigen::Quaterniond quat(Eigen::AngleAxis<double>(static_cast<double>(angle), Eigen::Vector3d::UnitZ()));
      pose_stamped.pose.orientation.x = quat.x();
      pose_stamped.pose.orientation.y = quat.y();
      pose_stamped.pose.orientation.z = quat.z();
      pose_stamped.pose.orientation.w = quat.w();

      // Create new place location
      moveit_msgs::PlaceLocation place_loc;

      place_loc.place_pose = pose_stamped;

      // Approach
      moveit_msgs::GripperTranslation pre_place_approach;
      pre_place_approach.direction.header.stamp = ros::Time::now();
      pre_place_approach.desired_distance = groups_[group]->grasp_data_.approach_retreat_desired_dist_; // The distance the origin of a robot link needs to travel
      pre_place_approach.min_distance = groups_[group]->grasp_data_.approach_retreat_min_dist_; // half of the desired? Untested.
      pre_place_approach.direction.header.frame_id = groups_[group]->grasp_data_.base_link_;
      pre_place_approach.direction.vector.x = 0;
      pre_place_approach.direction.vector.y = 0;
      pre_place_approach.direction.vector.z = -1; // Approach direction (negative z axis)  // TODO: document this assumption
      place_loc.pre_place_approach = pre_place_approach;

      // Retreat
      moveit_msgs::GripperTranslation post_place_retreat;
      post_place_retreat.direction.header.stamp = ros::Time::now();
      post_place_retreat.desired_distance = groups_[group]->grasp_data_.approach_retreat_desired_dist_; // The distance the origin of a robot link needs to travel
      post_place_retreat.min_distance = groups_[group]->grasp_data_.approach_retreat_min_dist_; // half of the desired? Untested.
      post_place_retreat.direction.header.frame_id = groups_[group]->grasp_data_.base_link_;
      post_place_retreat.direction.vector.x = 0;
      post_place_retreat.direction.vector.y = 0;
      post_place_retreat.direction.vector.z = 1; // Retreat direction (pos z axis)
      place_loc.post_place_retreat = post_place_retreat;

      // Post place posture - use same as pre-grasp posture (the OPEN command)
      place_loc.post_place_posture = groups_[group]->grasp_data_.pre_grasp_posture_;

      place_locations.push_back(place_loc);
    }

    // Prevent collision with table
    //move_group_->setSupportSurfaceName(SUPPORT_SURFACE3_NAME);

    if (groups_[group]->move_group_->place(groups_[group]->grasped_object_->id, place_locations))
    {

      groups_[group]->visual_tools_->cleanupCO(groups_[group]->grasped_object_->id);
      groups_[group]->visual_tools_->cleanupACO(groups_[group]->grasped_object_->id);
      groups_[group]->grasped_object_.reset();
      return true;
    }

    ROS_WARN("Failed to place");
    return false;
  }

  std::string getPlanningFrame(const planning_group &group)
  {

    return groups_[group]->move_group_->getPlanningFrame();

  }

  void publishCollisionBB(geometry_msgs::Pose block_pose, std::string block_name, const planning_group &group, const shape_msgs::SolidPrimitive & shape)
  {
    moveit_msgs::CollisionObject collision_obj;

    collision_obj.header.stamp = ros::Time::now();
    collision_obj.header.frame_id = getPlanningFrame(group);
    collision_obj.id = block_name;
    collision_obj.operation = moveit_msgs::CollisionObject::ADD;
    collision_obj.primitives.resize(1);
    collision_obj.primitives[0] = shape;
    collision_obj.primitive_poses.resize(1);
    collision_obj.primitive_poses[0] = block_pose;

    for (int j = 0; j < 3; j++)
    {

      groups_[group]->visual_tools_->cleanupCO(block_name);
      groups_[group]->visual_tools_->cleanupACO(block_name);
      ros::Duration(0.1).sleep();
    }

    for (int j = 0; j < 3; j++)
    {

      groups_[group]->visual_tools_->publishCollisionObjectMsg(collision_obj);
      ros::Duration(0.1).sleep();
    }

    return;
  }

  bool hasGraspedObject(const planning_group &group)
  {

    return groups_[group]->grasped_object_;
  }

  bool pick(const std::string &id, const planning_group &group, const geometry_msgs::Pose &ps, const shape_msgs::SolidPrimitive & shape)
  {

    if (hasGraspedObject(group))
    {

      ROS_ERROR("Can't grasp another object.");
      return false;
    }

    // todo add support for cylinder
    if (shape.type != shape_msgs::SolidPrimitive::BOX/* && shape.type != shape_msgs::SolidPrimitive::CYLINDER*/)
    {

      ROS_ERROR("Unsuported object type.");
      return false;
    }

    if (shape.dimensions.size() != 3)
    {

      ROS_ERROR("Strange dimensions!");
      return false;
    }

    lookAt(ps.position);

    publishCollisionBB(ps, id, group, shape);

    std::vector<moveit_msgs::Grasp> grasps;

    geometry_msgs::PoseStamped p;
    p.header.frame_id = getPlanningFrame(group);
    p.header.stamp = ros::Time::now();
    p.pose = ps;

    // visualization only
    groups_[group]->visual_tools_->publishBlock(ps, moveit_visual_tools::ORANGE, shape.dimensions[0], shape.dimensions[1], shape.dimensions[2]);

    if (!groups_[group]->simple_grasps_->generateShapeGrasps(shape, true, true, p, groups_[group]->grasp_data_, grasps))
    {

      ROS_ERROR("No grasps found.");
      return false;
    }

    std::vector<trajectory_msgs::JointTrajectoryPoint> ik_solutions; // save each grasps ik solution for visualization
    if (!groups_[group]->grasp_filter_->filterGrasps(grasps, ik_solutions, true, groups_[group]->grasp_data_.ee_parent_link_, group_name_[group] + "_arm"))
    {

      ROS_ERROR("Grasp filtering failed.");
      return false;
    }

    if (grasps.size() == 0)
    {

      ROS_ERROR("No feasible grasps found.");
      return false;
    }

    // visualization only - takes time
    /*if (!groups_[group]->visual_tools_->publishAnimatedGrasps(grasps, groups_[group]->grasp_data_.ee_parent_link_, 0.02)) {

        ROS_WARN("Grasp animation failed");
    }*/

    std::vector<std::string> allowed_touch_objects;
    allowed_touch_objects.push_back(id);

    // Add this list to all grasps
    for (std::size_t i = 0; i < grasps.size(); ++i)
    {
      grasps[i].allowed_touch_objects = allowed_touch_objects;
    }

    if (groups_[group]->move_group_->pick(id, grasps))
    {

      groups_[group]->grasped_object_.reset(new graspedObject());
      groups_[group]->grasped_object_->shape = shape;
      groups_[group]->grasped_object_->id = id;
      return true;
    }

    ROS_WARN("Failed to pick");
    return false;

  }

};

} // namespace art_pr2_grasping

#endif  // ART_PR2_GRASPING_PR2GRASP_H
