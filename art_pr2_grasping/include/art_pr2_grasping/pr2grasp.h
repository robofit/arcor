// Copyright 2016 Robo@FIT

#ifndef ART_PR2_GRASPING_PR2GRASP_H
#define ART_PR2_GRASPING_PR2GRASP_H

#include <map>
#include <string>
#include <vector>
#include <algorithm>
#include <ros/ros.h>
#include <moveit/robot_state/attached_body.h>
#include <eigen_stl_containers/eigen_stl_vector_container.h>
#include "tf/transform_datatypes.h"
#include <moveit/move_group_interface/move_group.h>
#include <moveit_simple_grasps/simple_grasps.h>
#include <moveit_visual_tools/visual_tools.h>
#include <moveit_simple_grasps/grasp_data.h>
#include <moveit_simple_grasps/grasp_filter.h>
#include "art_pr2_grasping/objects.h"
#include <std_msgs/String.h>
#include <std_msgs/Bool.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/Pose.h>
#include <pr2_controllers_msgs/JointControllerState.h>
#include "art_msgs/ObjInstance.h"

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

  moveit_simple_grasps::GraspFilterPtr grasp_filter_;
  planning_scene_monitor::PlanningSceneMonitorPtr planning_scene_monitor_;

  ros::Publisher grasped_object_pub_;
  ros::Publisher look_at_pub_;
  ros::Publisher place_pose_pub_;

  std::string default_target_;
  std::string gripper_state_topic_;

protected:
  boost::shared_ptr<tf::TransformListener> tfl_;

  boost::shared_ptr<move_group_interface::MoveGroup> move_group_;

  ros::NodeHandle nh_;

  std::string group_name_;

  boost::shared_ptr<Objects> objects_;
  boost::shared_ptr<TObjectInfo> grasped_object_;

  float getGripperValue();
  bool isRobotHalted();

  void look_at(const geometry_msgs::PoseStamped& ps);

  bool dont_try_again_;

public:
  artPr2Grasping(boost::shared_ptr<tf::TransformListener> tfl, boost::shared_ptr<Objects> objects,
                 std::string group_name, std::string default_target, std::string gripper_state_topic);

  bool transformPose(geometry_msgs::PoseStamped& ps);

  bool getReady();

  bool place(const geometry_msgs::Pose& ps, double z_axis_angle_increment = 0.0, bool keep_orientation = false);

  std::string getPlanningFrame();

  bool hasGraspedObject();

  void publishObject(TObjectInfo obj = TObjectInfo());

  bool pick(const std::string& object_id, bool feeder = false);

  bool addTable(std::string frame_id);
};
}  // namespace art_pr2_grasping
#endif  // ART_PR2_GRASPING_PR2GRASP_H
