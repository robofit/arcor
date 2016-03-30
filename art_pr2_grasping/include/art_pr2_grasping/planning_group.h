#include <string>
#include <ros/ros.h>
#include <moveit/move_group_interface/move_group.h>
#include <moveit_simple_grasps/simple_grasps.h>
#include <moveit_visual_tools/visual_tools.h>
#include <moveit_simple_grasps/grasp_data.h>
#include <moveit_simple_grasps/grasp_filter.h>

#ifndef ART_PR2_GRASPING_PLANNING_GROUP_H
#define ART_PR2_GRASPING_PLANNING_GROUP_H

namespace art_pr2_grasping
{

typedef struct
{

  shape_msgs::SolidPrimitive shape;
  std::string id;

} graspedObject;

class artPlanningGroup
{

public:
  moveit_simple_grasps::SimpleGraspsPtr simple_grasps_;

  moveit_visual_tools::VisualToolsPtr visual_tools_;

  // data for generating grasps
  moveit_simple_grasps::GraspData grasp_data_;

  // our interface with MoveIt
  boost::scoped_ptr<move_group_interface::MoveGroup> move_group_;

  std::string group_name_;

  ros::NodeHandle nh_;

  moveit_simple_grasps::GraspFilterPtr grasp_filter_;
  planning_scene_monitor::PlanningSceneMonitorPtr planning_scene_monitor_;

  // dimensions of the grasped object
  boost::shared_ptr<graspedObject> grasped_object_;

  explicit artPlanningGroup(std::string group_name)
    : nh_("~"),
      group_name_(group_name)
  {

    move_group_.reset(new move_group_interface::MoveGroup(group_name_ + "_arm"));
    move_group_->setPlanningTime(30.0);
    move_group_->allowLooking(false); // true causes failure
    move_group_->allowReplanning(true);
    move_group_->setGoalTolerance(0.005);
    move_group_->setPlannerId("RRTConnectkConfigDefault");

    // Load grasp generator
    if (!grasp_data_.loadRobotGraspData(nh_, group_name_))
      ros::shutdown();

    // Load the Robot Viz Tools for publishing to rviz
    visual_tools_.reset(new moveit_visual_tools::VisualTools(grasp_data_.base_link_, "markers"));
    visual_tools_->setFloorToBaseHeight(0.0);
    visual_tools_->loadEEMarker(grasp_data_.ee_group_, group_name_ + + "_arm");
    visual_tools_->loadPlanningSceneMonitor();
    visual_tools_->setLifetime(10.0);

    simple_grasps_.reset(new moveit_simple_grasps::SimpleGrasps(visual_tools_));

    planning_scene_monitor_.reset(new planning_scene_monitor::PlanningSceneMonitor("robot_description"));
    robot_state::RobotState robot_state = planning_scene_monitor_->getPlanningScene()->getCurrentState();
    grasp_filter_.reset(new moveit_simple_grasps::GraspFilter(robot_state, visual_tools_));

  }

};

} // namespace art_pr2_grasping

#endif  // ART_PR2_GRASPING_PLANNING_GROUP_H
