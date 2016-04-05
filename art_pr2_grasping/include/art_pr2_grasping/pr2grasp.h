#include <string>
#include <vector>
#include <ros/ros.h>
#include <pr2_controllers_msgs/PointHeadAction.h>
#include <actionlib/client/simple_action_client.h>
#include "art_object_recognizer_msgs/InstancesArray.h"
#include <tf/transform_listener.h>
#include <moveit/move_group_interface/move_group.h>
#include <moveit_simple_grasps/simple_grasps.h>
#include <moveit_visual_tools/visual_tools.h>
#include <moveit_simple_grasps/grasp_data.h>
#include <moveit_simple_grasps/grasp_filter.h>

// adapted from https://github.com/davetcoleman/baxter_cpp/blob/hydro-devel/baxter_pick_place/src/block_pick_place.cpp

#ifndef ART_PR2_GRASPING_PR2GRASP_H
#define ART_PR2_GRASPING_PR2GRASP_H

namespace art_pr2_grasping
{

typedef actionlib::SimpleActionClient<pr2_controllers_msgs::PointHeadAction> PointHeadClient;

typedef struct {

    std_msgs::Header h;
    shape_msgs::SolidPrimitive bb;
    geometry_msgs::Pose p;

} tobj;

typedef struct
{
  std::string id;
  // todo some other info?
} graspedObject;

class artPr2Grasping
{

private:

  moveit_simple_grasps::SimpleGraspsPtr simple_grasps_;

  moveit_visual_tools::VisualToolsPtr visual_tools_;

  // data for generating grasps
  moveit_simple_grasps::GraspData grasp_data_;

  boost::scoped_ptr<move_group_interface::MoveGroup> move_group_;

  moveit_simple_grasps::GraspFilterPtr grasp_filter_;
  planning_scene_monitor::PlanningSceneMonitorPtr planning_scene_monitor_;

  boost::scoped_ptr<PointHeadClient> head_;

  ros::NodeHandle nh_;

  bool enable_looking_;

  ros::Subscriber obj_sub_;

  std::map<std::string, tobj> objects_;

  boost::shared_ptr<tf::TransformListener> tfl_;

  boost::shared_ptr<graspedObject> grasped_object_;

  std::string group_name_;

public:
  artPr2Grasping()
    : nh_("~")
  {

    tfl_.reset(new tf::TransformListener());

    nh_.param("enable_looking", enable_looking_, false);
    nh_.param<std::string>("group_name", group_name_, "left_arm");

    move_group_.reset(new move_group_interface::MoveGroup(group_name_));
    move_group_->setPlanningTime(30.0);
    move_group_->allowLooking(false); // true causes failure
    move_group_->allowReplanning(true);
    move_group_->setGoalTolerance(0.005);
    move_group_->setPlannerId("RRTConnectkConfigDefault");

    ROS_INFO_STREAM("Planning frame: " << getPlanningFrame());

    // Load grasp generator
    if (!grasp_data_.loadRobotGraspData(nh_, group_name_))
      ros::shutdown();

    // Load the Robot Viz Tools for publishing to rviz
    visual_tools_.reset(new moveit_visual_tools::VisualTools(getPlanningFrame(), "markers"));
    //visual_tools_->setFloorToBaseHeight(0.0);
    visual_tools_->loadEEMarker(grasp_data_.ee_group_, group_name_);
    visual_tools_->loadPlanningSceneMonitor();
    visual_tools_->setLifetime(10.0);

    visual_tools_->setMuted(false);
    visual_tools_->publishRemoveAllCollisionObjects();
    grasped_object_.reset();

    simple_grasps_.reset(new moveit_simple_grasps::SimpleGrasps(visual_tools_));

    planning_scene_monitor_.reset(new planning_scene_monitor::PlanningSceneMonitor("robot_description"));
    robot_state::RobotState robot_state = planning_scene_monitor_->getPlanningScene()->getCurrentState();
    grasp_filter_.reset(new moveit_simple_grasps::GraspFilter(robot_state, visual_tools_));

    head_.reset(new PointHeadClient("/head_traj_controller/point_head_action", true));
    if (!head_->waitForServer(ros::Duration(5)))
    {

      ROS_WARN("Point head action not available!");
    }

    obj_sub_ = nh_.subscribe("/art_object_detector/object_filtered", 1, &artPr2Grasping::detectedObjectsCallback, this);

  }

  bool transformPose(geometry_msgs::PoseStamped &ps)
  {

    try
    {

      if (tfl_->waitForTransform(getPlanningFrame(), ps.header.frame_id, ps.header.stamp, ros::Duration(5)))
      {

        tfl_->transformPose(getPlanningFrame(), ps, ps);

      }
      else
      {

        ROS_ERROR_STREAM("Transform between" << getPlanningFrame() << "and " << ps.header.frame_id << " not available!");
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

  void detectedObjectsCallback(const art_object_recognizer_msgs::InstancesArrayConstPtr &msg) {

      ROS_INFO_ONCE("InstancesArray received");

      // remove outdated objects
      std::map<std::string, tobj>::iterator it;
      std::vector<std::string> ids_to_remove;
      for ( it = objects_.begin(); it != objects_.end(); it++ ) {

          bool found = false;

          for(int i = 0; i < msg->instances.size(); i++) {

              if (msg->instances[i].object_id == it->first) {

                  found = true;
                  break;

              }
          }

          if (!found) {

            // don't clear grasped objects
            if (grasped_object_ && grasped_object_->id == it->first) continue;

            ids_to_remove.push_back(it->first);

          }
      }

      for (int i = 0; i < ids_to_remove.size(); i++) {

          std::map<std::string, tobj>::iterator it;
          it = objects_.find(ids_to_remove[i]);
          objects_.erase(it);
          visual_tools_->cleanupCO(it->first);
      }

      // add and publish currently detected objects
      for(int i = 0; i < msg->instances.size(); i++) {

        if (msg->instances[i].bbox.type != shape_msgs::SolidPrimitive::BOX) continue;
        if (msg->instances[i].bbox.dimensions.size() != 3) continue;

        geometry_msgs::PoseStamped ps;

        ps.header = msg->header;
        ps.pose = msg->instances[i].pose;

        if (!transformPose(ps)) {

            ROS_WARN("Failed to transform object.");
            continue;
        }

        publishCollisionBB(ps.pose, msg->instances[i].object_id, msg->instances[i].bbox);

        tobj ob;
        ob.h = ps.header;
        ob.bb = msg->instances[i].bbox;
        ob.p = ps.pose;
        //std::cout << ob.p.position.x << " " << ob.p.position.y << " " << ob.p.position.z << " " << std::endl;
        objects_[msg->instances[i].object_id] = ob;

      }

  }

  bool isKnownObject(std::string id) {

      std::map<std::string, tobj>::iterator it = objects_.find(id);
      return it != objects_.end();
  }

  //! Points the high-def camera frame at a point in a given frame
  void lookAt(geometry_msgs::Point pt)
  {

    if (!enable_looking_) return;

    pr2_controllers_msgs::PointHeadGoal goal;
    geometry_msgs::PointStamped point;
    point.header.frame_id = getPlanningFrame();
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

  bool getReady()
  {

    geometry_msgs::PoseStamped ps;
    ps.header.frame_id = getPlanningFrame();
    ps.header.stamp = ros::Time::now();

    // todo specify as (configurable) joint target
    ps.pose.position.x = 0.093;
    ps.pose.position.y = 0.7;
    if (group_name_ == "right_arm") ps.pose.position.y *= -1.0; // todo HACK!!!
    ps.pose.position.z = 1.0;
    ps.pose.orientation.x = -0.001;
    ps.pose.orientation.y = 0.320;
    ps.pose.orientation.z = -0.001;
    ps.pose.orientation.w = 0.947;

    move_group_->clearPathConstraints();
    move_group_->setPoseTarget(ps);

    if (!move_group_->move())
    {

      ROS_WARN("Failed to get ready.");
      return false;

    }

    return true;
  }

  // todo moznost zadat jako PoseStamped?
  bool addTable(double x, double y, double angle, double width, double height, double depth, std::string name)
  {

    ROS_INFO("Adding table: %s", name.c_str());

    visual_tools_->cleanupCO(name);

    for (int j = 0; j < 3; j++) // hmm, sometimes the table is not added
    {
      if (!visual_tools_->publishCollisionTable(x, y, angle, width, height, depth, name)) return false;
      move_group_->setSupportSurfaceName(name);
      ros::Duration(1).sleep();
    }

    return true;
  }

  bool place(const geometry_msgs::Pose &ps, double z_axis_angle_increment = 0.0, bool keep_orientation = false)
  {

    if (!hasGraspedObject())
    {
      ROS_ERROR("No object to place.");
      return false;
    }

    lookAt(ps.position);

    std::vector<moveit_msgs::PlaceLocation> place_locations;

    geometry_msgs::PoseStamped pose_stamped;
    pose_stamped.header.frame_id = getPlanningFrame();
    pose_stamped.header.stamp = ros::Time::now();
    pose_stamped.pose = ps;

    visual_tools_->publishBlock(ps, moveit_visual_tools::ORANGE, objects_[grasped_object_->id].bb.dimensions[0], objects_[grasped_object_->id].bb.dimensions[1], objects_[grasped_object_->id].bb.dimensions[2]);

    if (z_axis_angle_increment < 0) z_axis_angle_increment *= -1.0; // only positive increment allowed
    if (z_axis_angle_increment == 0) z_axis_angle_increment = 2 * M_PI; // for 0 we only want one cycle (only given orientation)

    // Create 360 degrees of place location rotated around a center
    for (double angle = 0; angle < 2 * M_PI; angle += z_axis_angle_increment)
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
      pre_place_approach.desired_distance = grasp_data_.approach_retreat_desired_dist_; // The distance the origin of a robot link needs to travel
      pre_place_approach.min_distance = grasp_data_.approach_retreat_min_dist_; // half of the desired? Untested.
      pre_place_approach.direction.header.frame_id = grasp_data_.base_link_;
      pre_place_approach.direction.vector.x = 0;
      pre_place_approach.direction.vector.y = 0;
      pre_place_approach.direction.vector.z = -1; // Approach direction (negative z axis)  // TODO: document this assumption
      place_loc.pre_place_approach = pre_place_approach;

      // Retreat
      moveit_msgs::GripperTranslation post_place_retreat;
      post_place_retreat.direction.header.stamp = ros::Time::now();
      // todo is box_z always height of the object?
      // assume that robot holds the object in the middle of its height
      double des_dist = std::max(grasp_data_.approach_retreat_desired_dist_, 0.05 + 0.5*objects_[grasped_object_->id].bb.dimensions[shape_msgs::SolidPrimitive::BOX_Z]);

      post_place_retreat.desired_distance = des_dist; // The distance the origin of a robot link needs to travel -> depends on the object size
      post_place_retreat.min_distance = grasp_data_.approach_retreat_min_dist_; // half of the desired? Untested.
      post_place_retreat.direction.header.frame_id = grasp_data_.base_link_;
      post_place_retreat.direction.vector.x = 0;
      post_place_retreat.direction.vector.y = 0;
      post_place_retreat.direction.vector.z = 1; // Retreat direction (pos z axis)
      place_loc.post_place_retreat = post_place_retreat;

      // Post place posture - use same as pre-grasp posture (the OPEN command)
      place_loc.post_place_posture = grasp_data_.pre_grasp_posture_;

      place_locations.push_back(place_loc);
    }

    if (keep_orientation) {

        ROS_INFO("Applying orientation constraint...");

        moveit_msgs::OrientationConstraint ocm;
        ocm.link_name = move_group_->getEndEffectorLink();
        ocm.header.frame_id = getPlanningFrame();
        ocm.orientation = move_group_->getCurrentPose().pose.orientation;
        ocm.absolute_x_axis_tolerance = 0.2;
        ocm.absolute_y_axis_tolerance = 0.2;
        ocm.absolute_z_axis_tolerance = M_PI;
        ocm.weight = 1.0;

        moveit_msgs::Constraints c;
        c.orientation_constraints.push_back(ocm);

        move_group_->setPathConstraints(c);
    }

    // Prevent collision with table
    //move_group_->setSupportSurfaceName(SUPPORT_SURFACE3_NAME)

    if (move_group_->place(grasped_object_->id, place_locations))
    {
      visual_tools_->cleanupCO(grasped_object_->id);
      visual_tools_->cleanupACO(grasped_object_->id);
      grasped_object_.reset();
      move_group_->clearPathConstraints();
      return true;
    }

    ROS_WARN("Failed to place");
    return false;
  }

  std::string getPlanningFrame()
  {

    return move_group_->getPlanningFrame();

  }

  void publishCollisionBB(geometry_msgs::Pose block_pose, std::string block_name, const shape_msgs::SolidPrimitive & shape)
  {
    moveit_msgs::CollisionObject collision_obj;

    collision_obj.header.stamp = ros::Time::now();
    collision_obj.header.frame_id = getPlanningFrame();
    collision_obj.id = block_name;
    collision_obj.operation = moveit_msgs::CollisionObject::ADD; // todo param for update?
    collision_obj.primitives.resize(1);
    collision_obj.primitives[0] = shape;
    collision_obj.primitive_poses.resize(1);
    collision_obj.primitive_poses[0] = block_pose;

    for (int j = 0; j < 3; j++)
    {

      visual_tools_->cleanupCO(block_name);
      visual_tools_->cleanupACO(block_name);
      ros::Duration(0.1).sleep();
    }

    for (int j = 0; j < 3; j++)
    {

      visual_tools_->publishCollisionObjectMsg(collision_obj);
      ros::Duration(0.1).sleep();
    }

    return;
  }

  bool hasGraspedObject()
  {
   return grasped_object_;
  }

  bool pick(const std::string &id)
  {

    if (hasGraspedObject())
    {

      ROS_ERROR("Can't grasp another object.");
      return false;
    }

    std::vector<moveit_msgs::Grasp> grasps;

    lookAt(objects_[id].p.position);

    publishCollisionBB(objects_[id].p, id, objects_[id].bb);

    geometry_msgs::PoseStamped p;
    p.header.frame_id = getPlanningFrame();
    p.header.stamp = ros::Time::now();
    p.pose = objects_[id].p;

    // visualization only
    visual_tools_->publishBlock(objects_[id].p, moveit_visual_tools::ORANGE, objects_[id].bb.dimensions[0], objects_[id].bb.dimensions[1], objects_[id].bb.dimensions[2]);

    moveit_simple_grasps::GraspData gd;

    if (!simple_grasps_->generateShapeGrasps(objects_[id].bb, true, true, p, grasp_data_, grasps))
    {

      ROS_ERROR("No grasps found.");
      return false;
    }

    // todo fix this (No kinematic solver found)
    std::vector<trajectory_msgs::JointTrajectoryPoint> ik_solutions; // save each grasps ik solution for visualization
    if (!grasp_filter_->filterGrasps(grasps, ik_solutions, true, grasp_data_.ee_parent_link_, group_name_))
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

    if (move_group_->pick(id, grasps))
    {

      grasped_object_.reset(new graspedObject());
      grasped_object_->id = id;
      return true;
    }

    ROS_WARN("Failed to pick");
    return false;

  }

};

} // namespace art_pr2_grasping

#endif  // ART_PR2_GRASPING_PR2GRASP_H
