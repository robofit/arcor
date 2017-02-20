// Copyright 2016 Robo@FIT

#include <string>
#include <ros/ros.h>
#include <tf/transform_listener.h>
#include "art_msgs/InstancesArray.h"
#include "art_msgs/ObjInstance.h"
#include "art_msgs/ObjectType.h"
#include "art_msgs/getObjectType.h"
#include <boost/thread/recursive_mutex.hpp>

#ifndef ART_PR2_GRASPING_OBJECTS_H
#define ART_PR2_GRASPING_OBJECTS_H

namespace art_pr2_grasping
{

typedef struct
{
  std::string object_id;
  geometry_msgs::PoseStamped pose;
  art_msgs::ObjectType type;
}
TObjectInfo;

typedef std::map<std::string, TObjectInfo> TObjectMap;

class Objects
{
public:
    Objects(boost::shared_ptr<tf::TransformListener> tfl, string target_frame)
    {

        object_type_srv_ = nh_.serviceClient<art_msgs::getObjectType>("/art/db/object_type/get");
        obj_sub_ = nh_.subscribe("/art/object_detector/object_filtered", 1, &aObjects::detectedObjectsCallback, this);

        target_frame_ = target_frame;
    }

    bool isKnownObject(std::string id)
    {
      boost::recursive_mutex::scoped_lock lock(mutex_);

      TObjectMap::iterator it = objects_.find(id);
      return it != objects_.end();
    }

    TObjectInfo getObject(std::string object_id) {

        boost::recursive_mutex::scoped_lock lock(mutex_);

        if (isKnownObject(object_id))
        return objects_[object_id];
        else return TObjectInfo(); // TODO use exception?
    }

private:

    std::string objects_frame_id;
    TObjectMap objects;
    boost::recursive_mutex mutex_;

    boost::shared_ptr<tf::TransformListener> tfl_;
    ros::ServiceClient object_type_srv_;
    std::string target_frame_;
    ros::Subscriber obj_sub_;

    bool transformPose(geometry_msgs::PoseStamped &ps)
    {
      try
      {
        if (tfl_->waitForTransform(target_frame_, ps.header.frame_id, ps.header.stamp, ros::Duration(5)))
        {
          tfl_->transformPose(target_frame_, ps, ps);
        }
        else
        {
          ROS_ERROR_STREAM("Transform between" << target_frame_ << "and " << ps.header.frame_id << " not "
                                                                                                        "available!");
          return false;
        }
      }
      catch (tf::TransformException& ex)
      {
        ROS_ERROR("%s", ex.what());
        return false;
      }

      return true;
    }

    void detectedObjectsCallback(const art_msgs::InstancesArrayConstPtr &msg)
    {
      boost::recursive_mutex::scoped_try_lock lock(mutex_);

      if (!lock)
        return;

      // remove outdated objects
      TObjectMap::iterator it;
      std::vector<std::string> ids_to_remove;
      for (it = objects_.begin(); it != objects_.end(); ++it)
      {
        bool found = false;

        for (int i = 0; i < msg->instances.size(); i++)
        {
          if (msg->instances[i].object_id == it->first)
          {
            found = true;
            break;
          }
        }

        if (!found)
        {
          ids_to_remove.push_back(it->first);
        }
      }

      for (int i = 0; i < ids_to_remove.size(); i++)
      {
        TObjectMap::iterator it;
        it = objects_.find(ids_to_remove[i]);
        objects_.erase(it);
      }

      // add and publish currently detected objects
      for (int i = 0; i < msg->instances.size(); i++)
      {
        geometry_msgs::PoseStamped ps;

        ps.header = msg->header;
        ps.pose = msg->instances[i].pose;

        if (!transformPose(ps))
        {
          ROS_WARN("Failed to transform object.");
          continue;
        }

        if (isKnownObject(msg->instances[i].object_id))
        {
          objects_[msg->instances[i].object_id].pose = ps;
        }
        else
        {

          art_msgs::getObjectType srv;
          srv.request.name = msg->instances[i].object_type;

          if (!object_type_srv_.call(srv))
          {
              ROS_ERROR("Failed to call object_type service");
              continue;
          }

          TObjectInfo obj;
          obj.object_id = msg->instances[i].object_id;
          obj.pose = ps;
          objects_[msg->instances[i].object_id].type = srv.response.object_type;

        }

      // TODO callback - ids_to_remove

    }

}
}  // namespace art_pr2_grasping

#endif  // ART_PR2_GRASPING_NODE_H
