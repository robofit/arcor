// Copyright 2016 Robo@FIT

#include "art_pr2_grasping/pr2grasp.h"
#include <algorithm>
#include <string>
#include <vector>

namespace art_pr2_grasping
{

float artPr2Grasping::getGripperValue()
{
  pr2_controllers_msgs::JointControllerStateConstPtr msg =
      ros::topic::waitForMessage<pr2_controllers_msgs::JointControllerState>(
          gripper_state_topic_, ros::Duration(1));

  if (msg)
    return msg->process_value;
  else
    return 1000;  // TODO(ZdenekM): NaN / exception?
}

artPr2Grasping::artPr2Grasping(boost::shared_ptr<tf::TransformListener> tfl,
                               boost::shared_ptr<Objects> objects,
                               std::string group_name,
                               std::string default_target,
                               std::string gripper_state_topic)
    : group_name_(group_name), default_target_(default_target),
      gripper_state_topic_(gripper_state_topic), nh_("~")
{
  tfl_ = tfl;
  objects_ = objects;

  move_group_.reset(new move_group_interface::MoveGroup(group_name_));
  move_group_->setPlanningTime(30.0);
  move_group_->allowLooking(false);
  move_group_->allowReplanning(false);
  move_group_->setGoalTolerance(0.005);
  move_group_->setPlannerId("RRTConnectkConfigDefault");

  ROS_INFO_NAMED(group_name_, "Planning frame: %s", getPlanningFrame().c_str());

  // Load grasp generator
  if (!grasp_data_.loadRobotGraspData(nh_, group_name_))
    ros::shutdown();

  // Load the Robot Viz Tools for publishing to rviz
  visual_tools_.reset(
      new moveit_visual_tools::VisualTools(getPlanningFrame(), "markers"));
  // visual_tools_->setFloorToBaseHeight(0.0);
  visual_tools_->loadEEMarker(grasp_data_.ee_group_, group_name_);
  visual_tools_->loadPlanningSceneMonitor();
  visual_tools_->setLifetime(10.0);

  visual_tools_->setMuted(false);

  grasped_object_.reset();

  simple_grasps_.reset(new moveit_simple_grasps::SimpleGrasps(visual_tools_));

  planning_scene_monitor_.reset(
      new planning_scene_monitor::PlanningSceneMonitor("robot_description"));
  robot_state::RobotState robot_state =
      planning_scene_monitor_->getPlanningScene()->getCurrentState();
  grasp_filter_.reset(
      new moveit_simple_grasps::GraspFilter(robot_state, visual_tools_));

  grasped_object_pub_ = nh_.advertise<art_msgs::ObjInstance>(
      "/art/pr2/" + group_name_ + "/grasped_object", 1, true);

  look_at_pub_ = nh_.advertise<geometry_msgs::PointStamped>(
      "/art/pr2/look_at", 10);

  publishObject();
}

void artPr2Grasping::publishObject(TObjectInfo obj)
{
    art_msgs::ObjInstance inst;
    inst.object_id = obj.object_id;
    inst.object_type = obj.type.name;
    grasped_object_pub_.publish(inst);
}

bool artPr2Grasping::transformPose(geometry_msgs::PoseStamped& ps)
{
  try
  {
    if (tfl_->waitForTransform(getPlanningFrame(), ps.header.frame_id,
                               ps.header.stamp, ros::Duration(1.0)))
    {
      tfl_->transformPose(getPlanningFrame(), ps, ps);
    }
    else
    {
      ROS_ERROR_NAMED(group_name_, "Transform between %s and %s not available!",
                      getPlanningFrame().c_str(),
                      std::string(ps.header.frame_id).c_str());
      return false;
    }
  }
  catch (tf::TransformException& ex)
  {
    ROS_ERROR_NAMED(group_name_, "TF exception: %s",
                    std::string(ex.what()).c_str());
    return false;
  }

  return true;
}

bool artPr2Grasping::getReady()
{
  move_group_->clearPathConstraints();
  if (!move_group_->setNamedTarget(default_target_))
  {
    ROS_ERROR_NAMED(group_name_, "Unknown default target name: %s",
                    default_target_.c_str());
    return false;
  }

  if (!move_group_->move())
  {
    ROS_WARN_NAMED(group_name_, "Failed to get ready.");
    return false;
  }

  return true;
}

void artPr2Grasping::look_at(const geometry_msgs::PoseStamped& ps)
{
    geometry_msgs::PointStamped pt;
    pt.header = ps.header;
    pt.point = ps.pose.position;

    look_at_pub_.publish(pt);
}

bool artPr2Grasping::place(const geometry_msgs::Pose& ps,
                           double z_axis_angle_increment, bool keep_orientation)
{
  if (!hasGraspedObject())
  {
    ROS_ERROR_NAMED(group_name_, "No object to place.");
    return false;
  }

  std::vector<moveit_msgs::PlaceLocation> place_locations;

  geometry_msgs::PoseStamped pose_stamped;
  pose_stamped.pose = ps;
  pose_stamped.header.frame_id = getPlanningFrame();
  pose_stamped.header.stamp = ros::Time::now();

  look_at(pose_stamped);

  shape_msgs::SolidPrimitive bb = grasped_object_->type.bbox;

  visual_tools_->publishBlock(pose_stamped.pose, moveit_visual_tools::ORANGE,
                              bb.dimensions[0], bb.dimensions[1],
                              bb.dimensions[2]);

  if (z_axis_angle_increment < 0)
    z_axis_angle_increment *= -1.0;  // only positive increment allowed
  if (z_axis_angle_increment == 0)
    z_axis_angle_increment =
        2 * M_PI;  // for 0 we only want one cycle (only given orientation)

  // Create 360 degrees of place location rotated around a center
  for (double angle = 0; angle < 2 * M_PI; angle += z_axis_angle_increment)
  {
    geometry_msgs::PoseStamped pps = pose_stamped;

    // Orientation
    tf::Quaternion tfq2 = tf::createQuaternionFromYaw(angle);

    tf::Quaternion tfq1;
    tf::quaternionMsgToTF(pps.pose.orientation, tfq1);

    tf::quaternionTFToMsg(tfq1 * tfq2, pps.pose.orientation);

    // Create new place location
    moveit_msgs::PlaceLocation place_loc;

    place_loc.place_pose = pps;

    // Approach
    moveit_msgs::GripperTranslation pre_place_approach;
    pre_place_approach.direction.header.stamp = ros::Time::now();
    pre_place_approach.desired_distance =
        grasp_data_.approach_retreat_desired_dist_;  // The distance the origin
                                                     // of a robot link needs
                                                     // to travel
    pre_place_approach.min_distance =
        grasp_data_
            .approach_retreat_min_dist_;  // half of the desired? Untested.
    pre_place_approach.direction.header.frame_id = grasp_data_.base_link_;
    pre_place_approach.direction.vector.x = 0;
    pre_place_approach.direction.vector.y = 0;
    pre_place_approach.direction.vector.z = -1;  // Approach direction
                                                 // (negative z axis)
                                                 // document this assumption
    place_loc.pre_place_approach = pre_place_approach;

    // Retreat
    moveit_msgs::GripperTranslation post_place_retreat;
    post_place_retreat.direction.header.stamp = ros::Time::now();
    // TODO(ZdenekM): is box_z always height of the object?
    // assume that robot holds the object in the middle of its height

    double des_dist =
        std::max(grasp_data_.approach_retreat_desired_dist_,
                 0.1 +
                     0.5 *
                         grasped_object_->type.bbox
                             .dimensions[shape_msgs::SolidPrimitive::BOX_Z]);

    post_place_retreat.desired_distance =
        des_dist;  // The distance the origin of a robot link needs to travel
                   // -> depends on the object size
    post_place_retreat.min_distance =
        grasp_data_
            .approach_retreat_min_dist_;  // half of the desired? Untested.
    post_place_retreat.direction.header.frame_id = grasp_data_.base_link_;
    post_place_retreat.direction.vector.x = 0;
    post_place_retreat.direction.vector.y = 0;
    post_place_retreat.direction.vector.z = 1;  // Retreat direction (pos z axis)
    place_loc.post_place_retreat = post_place_retreat;

    // Post place posture - use same as pre-grasp posture (the OPEN command)
    place_loc.post_place_posture = grasp_data_.pre_grasp_posture_;

    place_locations.push_back(place_loc);
  }

  if (keep_orientation)
  {
    ROS_INFO_NAMED(group_name_, "Applying orientation constraint...");

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
  // move_group_->setSupportSurfaceName(SUPPORT_SURFACE3_NAME)

  if (move_group_->place(grasped_object_->object_id, place_locations))
  {
    grasped_object_.reset();
    publishObject();
    move_group_->clearPathConstraints();
    return true;
  }

  ROS_WARN_NAMED(group_name_, "Failed to place");
  return false;
}

std::string artPr2Grasping::getPlanningFrame()
{
  return move_group_->getPlanningFrame();
}

bool artPr2Grasping::hasGraspedObject() { return grasped_object_; }

bool artPr2Grasping::pick(const std::string& object_id, bool feeder)
{
  if (hasGraspedObject())
  {
    ROS_ERROR_NAMED(group_name_, "Can't grasp another object.");
    return false;
  }

  TObjectInfo obj;

  try
  {
    obj = objects_->getObject(object_id);
  }
  catch (const std::invalid_argument& e)
  {
    ROS_ERROR_NAMED(group_name_, "Unknown object_id: %s", object_id.c_str());
    return false;
  }

  std::vector<moveit_msgs::Grasp> grasps;

  geometry_msgs::PoseStamped p = obj.pose;
  p.header.stamp = ros::Time::now();

  if (!transformPose(p))
  {
    return false;
  }

  // visualization only -> published by publishCollisionBB
  // visual_tools_->publishBlock(objects_[id].p, moveit_visual_tools::ORANGE,
  // objects_[id].bb.dimensions[0], objects_[id].bb.dimensions[1],
  // objects_[id].bb.dimensions[2]);

  if (!simple_grasps_->generateShapeGrasps(obj.type.bbox, true, true, p,
                                           grasp_data_, grasps))
  {
    ROS_ERROR_NAMED(group_name_, "No grasps found.");
    return false;
  }

  look_at(p);
  objects_->setPaused(true);

  if (feeder) {

      // remove side (checked in object coordinates) and top grasps (checked in planning frame)

      std::vector<moveit_msgs::Grasp> tmp;

      for (int i = 0; i < grasps.size(); i++) {

        geometry_msgs::PoseStamped pp = grasps[i].grasp_pose;

          try
          {
            if (tfl_->waitForTransform("object_id_" + object_id, pp.header.frame_id,
                                       pp.header.stamp, ros::Duration(1.0)))
            {
              tfl_->transformPose("object_id_" + object_id, pp, pp);
            }
            else
            {
              continue;
            }
          }
          catch (tf::TransformException& ex)
          {
            ROS_ERROR_NAMED(group_name_, "TF exception: %s",
                            std::string(ex.what()).c_str());
            continue;
          }

        if (abs(pp.pose.position.z) < obj.type.bbox.dimensions[2]/2.0 && abs(grasps[i].grasp_pose.pose.position.z-obj.pose.pose.position.z) < obj.type.bbox.dimensions[0]/2) {

            tmp.push_back(grasps[i]);

        }
      }

      ROS_INFO_NAMED(group_name_, "%d grasps out of %d pruned out as not feasible for pick from feeder.", grasps.size() - tmp.size(), grasps.size());

      grasps = tmp;

  }

  // todo fix this (No kinematic solver found)
  std::vector<trajectory_msgs::JointTrajectoryPoint>
      ik_solutions;  // save each grasps ik solution for visualization
  if (!grasp_filter_->filterGrasps(grasps, ik_solutions, true,
                                   grasp_data_.ee_parent_link_, group_name_))
  {
    ROS_ERROR_NAMED(group_name_, "Grasp filtering failed.");
    return false;
  }

  if (grasps.size() == 0)
  {
    ROS_ERROR_NAMED(group_name_, "No feasible grasps found.");
    return false;
  }

  // visualization only - takes time
  /*if (!groups_[group]->visual_tools_->publishAnimatedGrasps(grasps,
  groups_[group]->grasp_data_.ee_parent_link_, 0.02)) {

      ROS_WARN("Grasp animation failed");
  }*/

  std::vector<std::string> allowed_touch_objects;
  allowed_touch_objects.push_back(object_id);

  // Add this list to all grasps
  for (std::size_t i = 0; i < grasps.size(); ++i)
  {
    grasps[i].allowed_touch_objects = allowed_touch_objects;
  }

  grasped_object_ = boost::make_shared<TObjectInfo>(obj);
  grasped_object_->object_id = object_id;

  if (!move_group_->pick(object_id, grasps))
  {
    ROS_WARN_NAMED(group_name_, "Failed to pick");
    grasped_object_.reset();
    return false;
  }

  float gripper = getGripperValue();

  if (gripper < 0.005)
  {
    visual_tools_->cleanupACO(object_id);
    grasped_object_.reset();
    ROS_ERROR_NAMED(group_name_,
                    "Gripper is closed - object missed or dropped :-(");
    return false;
  }

  ROS_INFO_NAMED(group_name_, "Picked the object.");
  publishObject(obj);
  return true;
}

// TODO(ZdenekM): move to Objects? Or somewhere else?
bool artPr2Grasping::addTable(std::string frame_id)
{

  visual_tools_->cleanupCO("table");

  geometry_msgs::PoseStamped ps;
  ps.header.frame_id = frame_id;
  ps.pose.orientation.w = 1.0;
  
  if (!transformPose(ps)) return false;

  geometry_msgs::PoseStamped k1;
  k1.header.frame_id = "n1_kinect2_link";
  k1.pose.orientation.w = 1.0;

  if (!transformPose(k1)) return false;

  geometry_msgs::PoseStamped k2; 
  k2.header.frame_id = "n2_kinect2_link";
  k2.pose.orientation.w = 1.0;

  if (!transformPose(k2)) return false;

  for (int j = 0; j < 3; j++)  // hmm, sometimes the table is not added
  {
      visual_tools_->publishCollisionTable(ps.pose.position.x-0.75/2, 0, 0, 1.5, ps.pose.position.z, 0.75, "table");
      // visual_tools_->publishCollisionTable(0.5, 1.1, 0, 0.1, 2.0, 1.0, "left-guard");
      // visual_tools_->publishCollisionTable(0.5, -1.1, 0, 0.1, 2.0, 1.0, "right-guard");
      visual_tools_->publishCollisionBlock(k1.pose, "kinect-n1", 0.3);
      visual_tools_->publishCollisionBlock(k2.pose, "kinect-n2", 0.3);

    move_group_->setSupportSurfaceName("table");
    ros::Duration(0.1).sleep();
  }

  return true;
}

}  // namespace art_pr2_grasping
