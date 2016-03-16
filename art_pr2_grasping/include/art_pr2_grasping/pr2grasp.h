#include <ros/ros.h>
#include <moveit/move_group_interface/move_group.h>
#include <moveit_simple_grasps/simple_grasps.h>
#include <moveit_visual_tools/visual_tools.h>
#include <moveit_simple_grasps/grasp_data.h>

// adapted from https://github.com/davetcoleman/baxter_cpp/blob/hydro-devel/baxter_pick_place/src/block_pick_place.cpp

namespace art_pr2_grasping {

  class artPr2Grasping {
  
    private:
    
      moveit_simple_grasps::SimpleGraspsPtr simple_grasps_;

      moveit_visual_tools::VisualToolsPtr visual_tools_;

      // data for generating grasps
      moveit_simple_grasps::GraspData grasp_data_;

      // our interface with MoveIt
      boost::scoped_ptr<move_group_interface::MoveGroup> move_group_;

      // which baxter arm are we using
      std::string arm_;
      std::string planning_group_name_;
      
      ros::NodeHandle nh_;
      
    public:
    
      artPr2Grasping()
        : arm_("left"),
          planning_group_name_(arm_ + "_arm"),
          nh_("~") {
          
          // Create MoveGroup for one of the planning groups
          move_group_.reset(new move_group_interface::MoveGroup(planning_group_name_));
          move_group_->setPlanningTime(30.0);

          // Load grasp generator
          if (!grasp_data_.loadRobotGraspData(nh_, arm_))
            ros::shutdown();

          // Load the Robot Viz Tools for publishing to rviz
          visual_tools_.reset(new moveit_visual_tools::VisualTools( grasp_data_.base_link_));
          visual_tools_->setFloorToBaseHeight(-0.9);
          visual_tools_->loadEEMarker(grasp_data_.ee_group_, planning_group_name_);

          simple_grasps_.reset(new moveit_simple_grasps::SimpleGrasps(visual_tools_));
          
          }
          
      
  
  };

};
