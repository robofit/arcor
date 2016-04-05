#include <string>
#include <vector>
#include "art_pr2_grasping/planning_group.h"
#include <pr2_controllers_msgs/PointHeadAction.h>
#include <actionlib/client/simple_action_client.h>
#include "art_object_recognizer_msgs/InstancesArray.h"
#include <tf/transform_listener.h>

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

class artPr2Grasping
{

private:
  static const int PLANNING_GROUPS = 1; // 2; -> TODO only left arm - for testing purposes

  boost::scoped_ptr<artPlanningGroup> groups_[PLANNING_GROUPS];
  std::string group_name_[PLANNING_GROUPS];
  boost::scoped_ptr<PointHeadClient> head_;

  ros::NodeHandle nh_;

  bool enable_looking_;

  ros::Subscriber obj_sub_;

  std::map<std::string, tobj> objects_;

  boost::shared_ptr<tf::TransformListener> tfl_;

public:
  artPr2Grasping()
    : nh_("~")
  {

    tfl_.reset(new tf::TransformListener());

    group_name_[LEFT] = "left";
    //group_name_[RIGHT] = "right";

    nh_.param("enable_looking", enable_looking_, false);

    // todo try to use "arms" group with appropriate end eff. name
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

    obj_sub_ = nh_.subscribe("/art_object_detector/object_filtered", 1, &artPr2Grasping::detectedObjectsCallback, this);

  }

  bool transformPose(std::string target_frame, geometry_msgs::PoseStamped &ps)
  {

    try
    {

      if (tfl_->waitForTransform(target_frame, ps.header.frame_id, ps.header.stamp, ros::Duration(5)))
      {

        tfl_->transformPose(target_frame, ps, ps);

      }
      else
      {

        ROS_ERROR_STREAM("Transform between" << target_frame << "and " << ps.header.frame_id << " not available!");
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
      for ( it = objects_.begin(); it != objects_.end(); it++ ) {

          bool found = false;

          for(int i = 0; i < msg->instances.size(); i++) {

              if (msg->instances[i].object_id == it->first) {

                  found = true;
                  break;

              }
          }

          if (!found) {

            // don't clear attached objects - they might be used
            //for (int i = 0; i < PLANNING_GROUPS; i++)
            //{
            groups_[0]->visual_tools_->cleanupCO(it->first);
            //}

          }
      }

      objects_.clear();

      // add and publish currently detected objects
      for(int i = 0; i < msg->instances.size(); i++) {

        if (msg->instances[i].bbox.type != shape_msgs::SolidPrimitive::BOX) continue;
        if (msg->instances[i].bbox.dimensions.size() != 3) continue;

        tobj ob;
        ob.h = msg->header;
        ob.bb = msg->instances[i].bbox;
        ob.p = msg->instances[i].pose;
        objects_[msg->instances[i].object_id] = ob;

        //for (int i = 0; i < PLANNING_GROUPS; i++)
        //{
        geometry_msgs::PoseStamped ps;

        ps.header = msg->header;
        ps.pose = msg->instances[i].pose;

        // assume that all groups have same planningFrame
        if (!transformPose(getPlanningFrame(0), ps)) {

            ROS_WARN("Failed to transform object.");
            continue;
        }

        publishCollisionBB(ps.pose, msg->instances[i].object_id, (planning_group)/*i*/0, msg->instances[i].bbox);
        //}

      }

  }

  bool isKnownObject(std::string id) {

      std::map<std::string, tobj>::iterator it = objects_.find(id);
      return it != objects_.end();
  }

  enum planning_group {LEFT, RIGHT};

  //! Points the high-def camera frame at a point in a given frame
  void lookAt(geometry_msgs::Point pt)
  {

    if (!enable_looking_) return;

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

    groups_[group]->move_group_->clearPathConstraints();
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

  bool place(const planning_group &group, const geometry_msgs::Pose &ps, double z_axis_angle_increment = 0.0, bool keep_orientation = false)
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

      // todo is box_z always height of the object?
      // assume that robot holds the object in the middle of its height
      double des_dist = std::max(groups_[group]->grasp_data_.approach_retreat_desired_dist_, 0.05 + 0.5*groups_[group]->grasped_object_->shape.dimensions[shape_msgs::SolidPrimitive::BOX_Z]);

      post_place_retreat.desired_distance = des_dist; // The distance the origin of a robot link needs to travel -> depends on the object size
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

    if (keep_orientation) {

        ROS_INFO("Applying orientation constraint...");

        moveit_msgs::OrientationConstraint ocm;
        ocm.link_name = groups_[group]->move_group_->getEndEffectorLink();
        ocm.header.frame_id = groups_[group]->move_group_->getPlanningFrame();
        ocm.orientation = groups_[group]->move_group_->getCurrentPose().pose.orientation;
        ocm.absolute_x_axis_tolerance = 0.2;
        ocm.absolute_y_axis_tolerance = 0.2;
        ocm.absolute_z_axis_tolerance = M_PI;
        ocm.weight = 1.0;

        moveit_msgs::Constraints c;
        c.orientation_constraints.push_back(ocm);

        groups_[group]->move_group_->setPathConstraints(c);
    }

    // Prevent collision with table
    //move_group_->setSupportSurfaceName(SUPPORT_SURFACE3_NAME);

    if (groups_[group]->move_group_->place(groups_[group]->grasped_object_->id, place_locations))
    {
      groups_[group]->visual_tools_->cleanupCO(groups_[group]->grasped_object_->id);
      groups_[group]->visual_tools_->cleanupACO(groups_[group]->grasped_object_->id);
      groups_[group]->grasped_object_.reset();
      groups_[group]->move_group_->clearPathConstraints();
      return true;
    }

    ROS_WARN("Failed to place");
    return false;
  }

  std::string getPlanningFrame(const planning_group &group)
  {

    return groups_[group]->move_group_->getPlanningFrame();

  }

  std::string getPlanningFrame(const int &group)
  {

    return getPlanningFrame((planning_group)group);

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

  bool pick(const std::string &id, const planning_group &group/*, const geometry_msgs::Pose &ps, const shape_msgs::SolidPrimitive & shape*/)
  {

    if (hasGraspedObject(group))
    {

      ROS_ERROR("Can't grasp another object.");
      return false;
    }

    std::vector<moveit_msgs::Grasp> grasps;

    geometry_msgs::PoseStamped ps;
    ps.header.frame_id = objects_[id].h.frame_id;
    ps.header.stamp = ros::Time(0);
    ps.pose = objects_[id].p;

    if (!transformPose(getPlanningFrame(group), ps)) {

        ROS_ERROR("Pick: transformation failed.");
        return false;
    }

    lookAt(ps.pose.position);

    publishCollisionBB(ps.pose, id, group, objects_[id].bb);

    geometry_msgs::PoseStamped p;
    p.header.frame_id = getPlanningFrame(group);
    p.header.stamp = ros::Time::now();
    p.pose = ps.pose;

    // visualization only
    groups_[group]->visual_tools_->publishBlock(ps.pose, moveit_visual_tools::ORANGE, objects_[id].bb.dimensions[0], objects_[id].bb.dimensions[1], objects_[id].bb.dimensions[2]);

    if (!groups_[group]->simple_grasps_->generateShapeGrasps(objects_[id].bb, true, true, p, groups_[group]->grasp_data_, grasps))
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
      groups_[group]->grasped_object_->shape = objects_[id].bb; // todo remember only id?
      groups_[group]->grasped_object_->id = id;
      return true;
    }

    ROS_WARN("Failed to pick");
    return false;

  }

};

} // namespace art_pr2_grasping

#endif  // ART_PR2_GRASPING_PR2GRASP_H
