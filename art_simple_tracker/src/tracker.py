#! /usr/bin/env python
import roslib; roslib.load_manifest('art_simple_tracker')
import rospy
from art_msgs.msg import InstancesArray, ObjInstance
import tf
from geometry_msgs.msg import Pose, PoseStamped
from math import sqrt
import numpy as np

# "tracking" of static objects
class tracker:

  def __init__(self, target_frame = "/marker"):
  
    self.target_frame = target_frame
    self.listener = tf.TransformListener()
    self.sub = rospy.Subscriber("/art_object_detector/object", InstancesArray, self.cb)
    self.pub = rospy.Publisher("/art_object_detector/object_filtered", InstancesArray)
    self.timer = rospy.Timer(rospy.Duration(1.0), self.timer_cb)
    self.objects = {}
    
    # should be in (0,1)
    self.ap = 0.25 # filtering cooeficient - position
    self.ao = 0.1 # filtering cooeficient - orientation
    
    self.min_cnt = 5 # publish object after it has been seen x times at least
    self.max_age = rospy.Duration(5)
  
  def timer_cb(self, event):
  
    ia = InstancesArray()
    ia.header.frame_id = self.target_frame
    ia.header.stamp = rospy.Time.now()
    
    objects_to_prune = []
    
    now = rospy.Time.now()
  
    for k, v in self.objects.iteritems():
    
      if (now - v["pose"].header.stamp) > self.max_age:
      
        objects_to_prune.append(k)
    
      if v["cnt"] < self.min_cnt:
        continue
        
      obj = ObjInstance()
      obj.pose = v["pose"].pose
      obj.pose.orientation = self.normalize(obj.pose.orientation)
      obj.bbox = v["bbox"]
      obj.object_id = k
      ia.instances.append(obj)
    
    # TODO also publish TF for each object???
    self.pub.publish(ia)
    
    for k in objects_to_prune:
    
      rospy.loginfo("Object " + k + " no longer visible")
      del self.objects[k]
      
    if len(objects_to_prune) > 0:
    
      rospy.loginfo("Pruned " + str(len(objects_to_prune)) + " objects")
  
  def normalize(self, q, tolerance=0.00001):
    v = (q.x, q.y, q.z, q.w)
    mag2 = sum(n * n for n in v)
    if abs(mag2 - 1.0) > tolerance:
        mag = sqrt(mag2)
        q.x /= mag
        q.y /= mag
        q.z /= mag
        q.w /= mag
    return q

  def transform(self, header, pose):
  
    ps = PoseStamped()
    ps.header = header
    ps.pose = pose
    
    if self.target_frame == header.frame_id: return ps

    if not self.listener.waitForTransform(self.target_frame, header.frame_id, header.stamp, rospy.Duration(4.0)):
    
      rospy.logwarn("Transform between " + self.target_frame + " and " + header.frame_id + " not available!")
      return None
      
    try:
      
      ps = self.listener.transformPose(self.target_frame, ps)
      
    except tf.Exception:
    
      rospy.logerr("TF exception")
      return None
      
    return ps
  
  def q2a(self, q):
  
    return [q.x, q.y, q.z, q.w]
  
  # here we assume that quaternions are close (object is static) -> averaging should be fine
  def filterPose(self, old, new):
  
    p = Pose()
    
    # check for q == -q and correct
    if (np.dot(self.q2a(old.orientation), self.q2a(new.orientation))) < 0.0:
    
        new.orientation.x *= -1.0
        new.orientation.y *= -1.0
        new.orientation.z *= -1.0
        new.orientation.w *= -1.0
    
    p.position.x = (1.0 - self.ap)*old.position.x + self.ap*new.position.x
    p.position.y = (1.0 - self.ap)*old.position.y + self.ap*new.position.y
    p.position.z = (1.0 - self.ap)*old.position.z + self.ap*new.position.z
    
    p.orientation.x = (1.0 - self.ao)*old.orientation.x + self.ao*new.orientation.x
    p.orientation.y = (1.0 - self.ao)*old.orientation.y + self.ao*new.orientation.y
    p.orientation.z = (1.0 - self.ao)*old.orientation.z + self.ao*new.orientation.z
    p.orientation.w = (1.0 - self.ao)*old.orientation.w + self.ao*new.orientation.w
    
    return p
    
  def cb(self, msg):
  
    for inst in msg.instances:
    
      ps = self.transform(msg.header, inst.pose)
        
      if ps is None:
      
        return
    
      if inst.object_id in self.objects:
      
        rospy.logdebug("Updating object: " + inst.object_id)
        
        self.objects[inst.object_id]["bbox"] = inst.bbox # should be same...
        self.objects[inst.object_id]["cnt"] += 1
        self.objects[inst.object_id]["pose"].header = ps.header
        self.objects[inst.object_id]["pose"].pose = self.filterPose(self.objects[inst.object_id]["pose"].pose, ps.pose)
        
      else:
      
        rospy.loginfo("Adding new object: " + inst.object_id)
        
        obj = {}
        obj["pose"] = ps
        obj["bbox"] = inst.bbox
        obj["cnt"] = 1
        
        self.objects[inst.object_id] = obj

if __name__ == '__main__':
    try:
        rospy.init_node('pp_client_py')
        tr = tracker()
        rospy.spin()
    except rospy.ROSInterruptException:
        print "program interrupted before completion"
