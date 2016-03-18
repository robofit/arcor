#include <ros/ros.h>
#include <moveit/move_group_interface/move_group.h>
#include <moveit_simple_grasps/simple_grasps.h>
#include <moveit_visual_tools/visual_tools.h>
#include <moveit_simple_grasps/grasp_data.h>
#include <moveit_simple_grasps/grasp_filter.h>
#include <actionlib/client/simple_action_client.h>
#include <pr2_controllers_msgs/PointHeadAction.h>
#include <actionlib/server/simple_action_server.h>
#include <art_pr2_grasping/pickplaceAction.h>
#include <tf/transform_listener.h>

// adapted from https://github.com/davetcoleman/baxter_cpp/blob/hydro-devel/baxter_pick_place/src/block_pick_place.cpp

namespace art_pr2_grasping {

  typedef actionlib::SimpleActionClient<pr2_controllers_msgs::PointHeadAction> PointHeadClient;

  class artPlanningGroup {

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
      float dim_; // todo udelat map: id/dim[]?

      artPlanningGroup(std::string group_name)
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
          visual_tools_.reset(new moveit_visual_tools::VisualTools( grasp_data_.base_link_, "markers"));
          visual_tools_->setFloorToBaseHeight(0.0);
          visual_tools_->loadEEMarker(grasp_data_.ee_group_, group_name_ + + "_arm");
          visual_tools_->loadPlanningSceneMonitor();
          visual_tools_->setLifetime(10.0);

          simple_grasps_.reset(new moveit_simple_grasps::SimpleGrasps(visual_tools_));

          planning_scene_monitor_.reset(new planning_scene_monitor::PlanningSceneMonitor("robot_description"));
          robot_state::RobotState robot_state = planning_scene_monitor_->getPlanningScene()->getCurrentState();
          grasp_filter_.reset(new moveit_simple_grasps::GraspFilter(robot_state, visual_tools_) );

      }

  };

  class artPr2Grasping {
  
    private:

      static const int PLANNING_GROUPS = 2;

      boost::scoped_ptr<artPlanningGroup> groups_[PLANNING_GROUPS];
      std::string group_name_[PLANNING_GROUPS];
      boost::scoped_ptr<PointHeadClient> head_;

      ros::NodeHandle nh_;

    public:
    
      artPr2Grasping()
        : nh_("~") {

          group_name_[LEFT] = "left";
          group_name_[RIGHT] = "right";

          for (int i = 0; i < PLANNING_GROUPS; i++) {

              groups_[i].reset(new artPlanningGroup(group_name_[i]));
              groups_[i]->visual_tools_->setMuted(false);
              groups_[i]->visual_tools_->publishRemoveAllCollisionObjects();
          }

          head_.reset(new PointHeadClient("/head_traj_controller/point_head_action", true));
          if (!head_->waitForServer(ros::Duration(5))) {

              ROS_WARN("Point head action not available!");
          }

      }

      enum planning_group {LEFT, RIGHT};

      //! Points the high-def camera frame at a point in a given frame
      void lookAt(geometry_msgs::Point pt)
        {
          //the goal message we will be sending
          pr2_controllers_msgs::PointHeadGoal goal;

          //the target point, expressed in the requested frame
          geometry_msgs::PointStamped point;
          point.header.frame_id = "base_footprint";
          point.point = pt;
          goal.target = point;
          goal.pointing_frame = "high_def_frame"; // todo zmenit na kinect
          goal.pointing_axis.x = 1;
          goal.pointing_axis.y = 0;
          goal.pointing_axis.z = 0;
          goal.min_duration = ros::Duration(0.5);
          goal.max_velocity = 0.5;
          head_->sendGoal(goal);
          head_->waitForResult(ros::Duration(2));
        }

      bool getReady(const planning_group & group) {

          geometry_msgs::PoseStamped ps;
          ps.header.frame_id = groups_[group]->grasp_data_.base_link_;
          ps.header.stamp = ros::Time::now();

          ps.pose.position.x = 0.093;
          ps.pose.position.y = 0.7;
          ps.pose.position.z = 1.0;
          ps.pose.orientation.x = -0.001;
          ps.pose.orientation.y = 0.320;
          ps.pose.orientation.z = -0.001;
          ps.pose.orientation.w = 0.947;

          if (group == RIGHT) ps.pose.position.y = -0.7;

          groups_[group]->move_group_->setPoseTarget(ps);
          return groups_[group]->move_group_->move();
      }

      bool getReady() {

          for (int i = 0; i < PLANNING_GROUPS; i++) {

            if (!getReady((planning_group)i)) return false;
          }

          return true;

      }

      // todo moznost zadat jako PoseStamped?
      bool addTable(double x, double y, double angle, double width, double height, double depth, std::string name) {

          ROS_INFO("Adding table: %s", name.c_str());

          for (int i = 0; i < PLANNING_GROUPS; i++) {

              groups_[i]->visual_tools_->cleanupCO(name);
              if (!groups_[i]->visual_tools_->publishCollisionTable(x, y, angle, width, height, depth, name)) return false;
              groups_[i]->move_group_->setSupportSurfaceName(name);
              ros::Duration(1).sleep();
          }

          return true;

      }

      // todo remove id? - the robot should know what's in his hand
      bool place(const std::string &id, const planning_group &group, const geometry_msgs::Pose &ps) {

          lookAt(ps.position);

          //groups_[group]->visual_tools_->publishBlock(ps.pose, moveit_visual_tools::ORANGE, 0.05);

          std::vector<moveit_msgs::PlaceLocation> place_locations;

          // todo transform ps into grasp_data_.base_link_

          geometry_msgs::PoseStamped pose_stamped;
          pose_stamped.header.frame_id = groups_[group]->grasp_data_.base_link_;
          pose_stamped.header.stamp = ros::Time::now();


          // Create 360 degrees of place location rotated around a center
          for (double angle = 0; angle < 2*M_PI; angle += M_PI/2)
          {
            pose_stamped.pose = ps;

            // Orientation
            Eigen::Quaterniond quat(Eigen::AngleAxis<double>(double(angle), Eigen::Vector3d::UnitZ()));
            pose_stamped.pose.orientation.x = quat.x();
            pose_stamped.pose.orientation.y = quat.y();
            pose_stamped.pose.orientation.z = quat.z();
            pose_stamped.pose.orientation.w = quat.w();

            // Create new place location
            moveit_msgs::PlaceLocation place_loc;

            place_loc.place_pose = pose_stamped;

            groups_[group]->visual_tools_->publishBlock( place_loc.place_pose.pose, moveit_visual_tools::ORANGE, groups_[group]->dim_);

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

          return groups_[group]->move_group_->place(id, place_locations);

      }

      std::string getPlanningFrame(const planning_group &group) {

          return groups_[group]->move_group_->getPlanningFrame();

      }

      bool pick(const std::string &id, const planning_group &group, const geometry_msgs::Pose &ps, const shape_msgs::SolidPrimitive & shape) {

          lookAt(ps.position);

          // todo check if something was picked-up before

          groups_[group]->visual_tools_->cleanupCO(id);
          groups_[group]->visual_tools_->cleanupACO(id);

          // todo check type of the object
          // todo zapamatovat si id/shape
          if (shape.dimensions.size() != 3) {

              ROS_ERROR("shape with no dimensions!");
              return false;
          }

          groups_[group]->dim_ = shape.dimensions[0];
          for (int i = 1; i < 3; i++) {

              if (groups_[group]->dim_ < shape.dimensions[i]) groups_[group]->dim_ = shape.dimensions[i];
          }

          //groups_[group]->visual_tools_->publishBlock(ps, moveit_visual_tools::BLUE, groups_[group]->dim_);
          // todo use BB instead of block
          groups_[group]->visual_tools_->publishCollisionBlock(ps, id, groups_[group]->dim_);

          std::vector<moveit_msgs::Grasp> grasps;

          // todo use generateBoxGrasps - nutno zkompilovat z gitu + vyresit dependency hell
          if (!groups_[group]->simple_grasps_->generateBlockGrasps( ps, groups_[group]->grasp_data_, grasps )) {

              ROS_ERROR("No grasps found.");
              return false;
          }

          std::vector<trajectory_msgs::JointTrajectoryPoint> ik_solutions; // save each grasps ik solution for visualization
          if (!groups_[group]->grasp_filter_->filterGrasps(grasps, ik_solutions, true, groups_[group]->grasp_data_.ee_parent_link_, group_name_[group] + "_arm")) {

              ROS_ERROR("Grasp filtering failed.");
              return false;
          }

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

          return groups_[group]->move_group_->pick(id, grasps);
      }
  
  };

  class artActionServer {

  public:

      artActionServer(): nh_("~"),
          as_(nh_, "pp", boost::bind(&artActionServer::executeCB, this, _1), false),
          max_attempts_(3)
      {

      }

      bool init() {

        // todo - tohle poradi (ready, addtable) je dobre pro simulaci - v realu by to bylo lepsi naopak??
        if (!gr_.getReady()) return false;

        ros::Duration(1).sleep();

        // todo - read from param?
        if (!gr_.addTable(0.75, 0, 0, 1.5, 0.74, 0.80, "table1")) {

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

      void executeCB(const pickplaceGoalConstPtr &goal) {

        pickplaceResult res;
        pickplaceFeedback f;
        artPr2Grasping::planning_group g;

        if (goal->arm == pickplaceGoal::LEFT_ARM) g = artPr2Grasping::LEFT;
        else if (goal->arm == pickplaceGoal::RIGHT_ARM) g = artPr2Grasping::RIGHT;
        else {

            res.result = pickplaceResult::BAD_REQUEST;
            as_.setAborted(res, "unknown planning group");
            return;
        }

        if (goal->operation != pickplaceGoal::PICK_AND_PLACE && goal->operation != pickplaceGoal::PICK && goal->operation != pickplaceGoal::PLACE) {

            res.result = pickplaceResult::BAD_REQUEST;
            as_.setAborted(res, "unknown operation");
            return;
        }

        ROS_INFO("Got goal for group: %d, operation: %d", goal->arm, goal->operation);

        if (goal->operation == pickplaceGoal::PICK_AND_PLACE || goal->operation == pickplaceGoal::PICK) {

            if (goal->bb.dimensions.size() != 3) {

                res.result = pickplaceResult::BAD_REQUEST;
                as_.setAborted(res, "for pick/p&p a bounding box must be given ");
                return;
            }

            int tries = max_attempts_;

            geometry_msgs::PoseStamped pst;

            if (goal->pose.header.frame_id == gr_.getPlanningFrame(g)) pst = goal->pose;
            else {

                bool err = false;

                try {
                    if (tfl_.waitForTransform(gr_.getPlanningFrame(g), goal->pose.header.frame_id, goal->pose.header.stamp, ros::Duration(5))) {
                        tfl_.transformPose(gr_.getPlanningFrame(g), goal->pose, pst);
                    } else err = true;
                } catch(tf::TransformException ex) {
                    ROS_ERROR("%s",ex.what());
                    err = true;
                }

                if (err) {

                    res.result = pickplaceResult::FAILURE;
                    as_.setAborted(res, "failed to transform pose");
                    return;
                }

            }

            f.operation = goal->operation == pickplaceGoal::PICK;

            while (tries > 0 && !as_.isPreemptRequested()) {

                f.attempt = (max_attempts_ - tries) + 1;
                ROS_INFO("Pick %d", f.attempt);
                tries--;
                as_.publishFeedback(f);
                if (gr_.pick(goal->id, g, pst.pose, goal->bb)) break;
            }

            if (as_.isPreemptRequested()) {

                gr_.getReady(g);
                as_.setPreempted(res, "pick cancelled");
                return;
            }

            if (tries == 0) {

                gr_.getReady(g);
                res.result = pickplaceResult::FAILURE;
                as_.setAborted(res);
                return;
            }

            if (goal->operation == pickplaceGoal::PICK) gr_.getReady(g); // get ready and wait with object grasped

        }

        if (goal->operation == pickplaceGoal::PICK_AND_PLACE || goal->operation == pickplaceGoal::PLACE) {

            int tries = max_attempts_;

            geometry_msgs::PoseStamped pst;

            if (goal->pose2.header.frame_id == gr_.getPlanningFrame(g)) pst = goal->pose2;
            else {

                bool err = false;

                try {
                    if (tfl_.waitForTransform(gr_.getPlanningFrame(g), goal->pose2.header.frame_id, goal->pose.header.stamp, ros::Duration(5))) {
                        tfl_.transformPose(gr_.getPlanningFrame(g), goal->pose2, pst);
                    } else err = true;
                } catch(tf::TransformException ex) {
                    ROS_ERROR("%s",ex.what());
                    err = true;
                }

                if (err) {

                    res.result = pickplaceResult::FAILURE;
                    as_.setAborted(res, "failed to transform pose");
                    return;
                }

            }

            f.operation = goal->operation == pickplaceGoal::PLACE;

            while (tries > 0 && !as_.isPreemptRequested()) {

                f.attempt = (max_attempts_ - tries) + 1;
                ROS_INFO("Place %d", f.attempt);
                tries--;
                as_.publishFeedback(f);
                if (gr_.place(goal->id, g, pst.pose)) break;
            }

            if (as_.isPreemptRequested()) {

                gr_.getReady(g);
                as_.setPreempted(res, "place cancelled");
                return;
            }

            if (tries == 0) {

                gr_.getReady(g);
                res.result = pickplaceResult::FAILURE;
                as_.setAborted(res);
                return;
            }

            gr_.getReady(g);

        }

        res.result = pickplaceResult::SUCCESS;
        as_.setSucceeded(res);

        }

      };


}
