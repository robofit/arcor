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
      // todo vector/map of groups?
      boost::scoped_ptr<move_group_interface::MoveGroup> move_group_l_;
      boost::scoped_ptr<move_group_interface::MoveGroup> move_group_r_;
      
      ros::NodeHandle nh_;
      
    public:
    
      artPr2Grasping()
        : nh_("~") {
          
          // Create MoveGroup for one of the planning groups
          move_group_l_.reset(new move_group_interface::MoveGroup("left_arm"));
          move_group_l_->setPlanningTime(30.0);
          move_group_l_->allowLooking(false); // true causes failure
          move_group_l_->allowReplanning(true);
          move_group_l_->setGoalTolerance(0.005);
          move_group_l_->setPlannerId("RRTConnectkConfigDefault");

          move_group_r_.reset(new move_group_interface::MoveGroup("right_arm"));
          move_group_r_->setPlanningTime(30.0);
          move_group_r_->allowLooking(false); // true causes failure
          move_group_r_->allowReplanning(true);
          move_group_r_->setGoalTolerance(0.005);
          move_group_r_->setPlannerId("RRTConnectkConfigDefault");

          // Load grasp generator
          if (!grasp_data_.loadRobotGraspData(nh_, "left"))
            ros::shutdown();

          // Load the Robot Viz Tools for publishing to rviz
          visual_tools_.reset(new moveit_visual_tools::VisualTools( grasp_data_.base_link_, "markers"));
          visual_tools_->setFloorToBaseHeight(-0.9);
          visual_tools_->loadEEMarker(grasp_data_.ee_group_, "left_arm");

          simple_grasps_.reset(new moveit_simple_grasps::SimpleGrasps(visual_tools_));

          }

      bool getReady() {

          //move_group_->setJointTarget(nejaka_vychozi_pozice)
          move_group_l_->setPoseTarget(move_group_l_->getRandomPose());
          move_group_l_->move();
          move_group_r_->setPoseTarget(move_group_r_->getRandomPose());
          move_group_r_->move();
          return true;

      }

      // todo remove id - the robot should know what's in his hand
      bool place(const std::string &id, const std::string &group, const geometry_msgs::PoseStamped &ps) {

          std::vector<moveit_msgs::PlaceLocation> place_locations;
          std::vector<moveit_msgs::Grasp> grasps;

          // todo transform ps into grasp_data_.base_link_

          // Re-usable datastruct
          geometry_msgs::PoseStamped pose_stamped;
          pose_stamped.header.frame_id = grasp_data_.base_link_;
          pose_stamped.header.stamp = ros::Time::now();


          // Create 360 degrees of place location rotated around a center
          for (double angle = 0; angle < 2*M_PI; angle += M_PI/2)
          {
            pose_stamped.pose = ps.pose;

            // Orientation
            Eigen::Quaterniond quat(Eigen::AngleAxis<double>(double(angle), Eigen::Vector3d::UnitZ()));
            pose_stamped.pose.orientation.x = quat.x();
            pose_stamped.pose.orientation.y = quat.y();
            pose_stamped.pose.orientation.z = quat.z();
            pose_stamped.pose.orientation.w = quat.w();

            // Create new place location
            moveit_msgs::PlaceLocation place_loc;

            place_loc.place_pose = pose_stamped;

            visual_tools_->publishBlock( place_loc.place_pose.pose, moveit_visual_tools::BLUE, 0.05);

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
            post_place_retreat.desired_distance = grasp_data_.approach_retreat_desired_dist_; // The distance the origin of a robot link needs to travel
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

          // Prevent collision with table
          //move_group_->setSupportSurfaceName(SUPPORT_SURFACE3_NAME);

          return move_group_l_->place(id, place_locations);

      }

      bool pick(const std::string &id, const std::string &group, const geometry_msgs::PoseStamped &ps/*, const shape_msgs::SolidPrimitive & shape*/) {

          visual_tools_->setMuted(false);
          visual_tools_->cleanupCO(id);
          visual_tools_->cleanupACO(id);

          visual_tools_->publishBlock(ps.pose, moveit_visual_tools::BLUE, 0.05);
          visual_tools_->publishCollisionBlock(ps.pose, id, 0.05); // todo ps transformovat do grasp_data_.base_link_

          std::vector<moveit_msgs::Grasp> grasps;

          // todo use generateBoxGrasps ?
          if (!simple_grasps_->generateBlockGrasps( ps.pose, grasp_data_, grasps )) {

              ROS_ERROR("No grasps found.");
              return false;
          }

          /*if (!visual_tools_->publishGrasps(grasps, grasp_data_.ee_parent_link_)) {

              ROS_WARN("Grasp animation failed");
          }*/

          std::vector<std::string> allowed_touch_objects;
          allowed_touch_objects.push_back("id");

          // Add this list to all grasps
          for (std::size_t i = 0; i < grasps.size(); ++i)
          {
            grasps[i].allowed_touch_objects = allowed_touch_objects;
          }

          return move_group_l_->pick(id, grasps);
      }
  
  };

};
