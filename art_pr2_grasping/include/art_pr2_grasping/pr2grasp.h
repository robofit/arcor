// Copyright 2016 Robo@FIT

#ifndef ART_PR2_GRASPING_PR2GRASP_H
#define ART_PR2_GRASPING_PR2GRASP_H

#include <map>
#include <string>
#include <vector>
#include <algorithm>
#include <ros/ros.h>
#include <moveit/move_group_interface/move_group.h>
#include <moveit_simple_grasps/simple_grasps.h>
#include <moveit_visual_tools/visual_tools.h>
#include <moveit_simple_grasps/grasp_data.h>
#include <moveit_simple_grasps/grasp_filter.h>
#include "art_pr2_grasping/objects.h"
#include <std_msgs/String.h>
#include <geometry_msgs/PointStamped.h>
#include <pr2_controllers_msgs/JointControllerState.h>

// adapted from
// https://github.com/davetcoleman/baxter_cpp/blob/hydro-devel/baxter_pick_place/src/block_pick_place.cpp

namespace art_pr2_grasping
{
class artPr2Grasping
{
private:
  moveit_simple_grasps::SimpleGraspsPtr simple_grasps_;

  moveit_visual_tools::VisualToolsPtr visual_tools_;

  // data for generating grasps
  moveit_simple_grasps::GraspData grasp_data_;

  boost::shared_ptr<move_group_interface::MoveGroup> move_group_;

  moveit_simple_grasps::GraspFilterPtr grasp_filter_;
  planning_scene_monitor::PlanningSceneMonitorPtr planning_scene_monitor_;

  ros::Publisher grasped_object_pub_;
  ros::Publisher look_at_pub_;

  boost::shared_ptr<tf::TransformListener> tfl_;

  std::string default_target_;
  std::string gripper_state_topic_;

  void publishObject(std::string object_id = "");

protected:
  ros::NodeHandle nh_;

  std::string group_name_;

  boost::shared_ptr<Objects> objects_;
  boost::shared_ptr<TObjectInfo> grasped_object_;

  float getGripperValue();
  
  void look_at(const geometry_msgs::PoseStamped& ps);

public:
  artPr2Grasping(boost::shared_ptr<tf::TransformListener> tfl,
                 boost::shared_ptr<Objects> objects, std::string group_name,
                 std::string default_target, std::string gripper_state_topic);

  bool transformPose(geometry_msgs::PoseStamped& ps);

  bool getReady();

  bool place(const geometry_msgs::Pose& ps, double z_axis_angle_increment = 0.0,
             bool keep_orientation = false);

  std::string getPlanningFrame();

  bool hasGraspedObject();

  bool pick(const std::string& object_id);

  bool addTable(double x, double y, double angle, double width, double height,
                double depth, std::string name);
};
}  // namespace art_pr2_grasping
#endif  // ART_PR2_GRASPING_PR2GRASP_H
