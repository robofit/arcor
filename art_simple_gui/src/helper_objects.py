import rospy
from PyQt4 import QtGui, QtCore
from geometry_msgs.msg import PoseStamped

class scene_place():
    
    def __init__(self,  scene, pos,  box_size,  pub):
        
        self.box_size = box_size
        self.scene = scene
        self.pos = pos
        self.pub = pub
        self.viz = self.scene.addEllipse(0, 0, 100, 100, QtCore.Qt.cyan, QtCore.Qt.cyan)
        self.viz.setPos(self.pos[0] - 100/2, self.pos[1] - 100/2)
        
        ps = self.get_pose(self.pos[0],  self.pos[1])
        self.pub.publish(ps)
     
    def remove(self):
        
        if self.viz is not None:
            self.scene.removeItem(self.viz)
            self.viz = None
  
    def get_pose(self, px, py):

        ps = PoseStamped()
        ps.header.frame_id = "marker"
        ps.header.stamp = rospy.Time.now()
        ps.pose.position.x = (self.scene.width() - px)*self.box_size/(self.scene.height()/10.0)
        ps.pose.position.y = py*self.box_size/(self.scene.height()/10.0)
        ps.pose.position.z = 0.0
        ps.pose.orientation.x = 0.0
        ps.pose.orientation.y = 0.0
        ps.pose.orientation.z = 0.0
        ps.pose.orientation.w = 1.0
        
        return ps
        
class scene_object():

    def __init__(self,  scene,  id, pos,  pub):
        
        self.scene = scene
        self.id = id
        self.pub = pub
        
        self.viz = self.scene.addEllipse(0, 0, 150, 150, QtCore.Qt.white, QtCore.Qt.white)
        
        self.label = self.scene.addText(id, QtGui.QFont('Arial', 16))
        self.label.setParentItem(self.viz)
        self.label.rotate(180)
        self.label.setDefaultTextColor(QtCore.Qt.white)
        
        self.set_pos(pos)
        
        self.pointed_at = None
        
        self.preselected = False
        self.preselected_at = None
        
        self.selected = False
        self.selected_at = None
        
        self.viz_preselect = None
        
        self.viz_selected = None
        
        self.timer = QtCore.QTimer()
        self.timer.start(500)
        self.timer.timeout.connect(self.timer_evt)
    
    def set_pos(self,  pos):
        
        self.pos = pos
        self.viz.setPos(self.pos[0] - 150/2, self.pos[1] - 150/2)
        self.label.setPos(120,  -10)
    
    def timer_evt(self):
        
        # cancel preselect after some time - object was not pointed at
        if not self.selected:
           
          if self.preselected and rospy.Time.now() - self.pointed_at  > rospy.Duration(0.5):
              
              rospy.loginfo("object " + self.id + ": preselect cancelled")
              self.preselected = False
              self.scene.removeItem(self.viz_preselect)
              self.viz_preselect = None
    
    def remove(self):
        
        self.scene.removeItem(self.viz)
        self.scene.removeItem(self.label)
        self.viz = None
        self.label = None
        
    def unselect(self):
        
         if self.preselected:
             
            self.preselected = False
            if self.viz_preselect is not None:
                self.scene.removeItem(self.viz_preselect)
                self.viz_preselect = None
            
         if self.selected:
            
            self.selected = False
            self.scene.removeItem(self.viz_selected)
            self.viz_selected = None
        
    def pointing(self,  pointing_obj):
        
        if self.viz not in pointing_obj.collidingItems(): return
        
        self.pointed_at = rospy.Time.now()
        
        if not self.preselected:
            
            rospy.loginfo("object " + self.id + ": preselected")
            
            self.preselected = True
            self.preselected_at = rospy.Time.now()
        
            self.viz_preselect = self.scene.addEllipse(0, 0, 180, 180, QtCore.Qt.gray, QtCore.Qt.gray)
            self.viz_preselect.setParentItem(self.viz)
            self.viz_preselect.setPos(150/2 - 180/2, 150/2 - 180/2)
            self.viz_preselect.setFlag(QtGui.QGraphicsItem.ItemStacksBehindParent)
            
            return
        
        if not self.selected:
           
          if rospy.Time.now() - self.preselected_at > rospy.Duration(1): self.selected = True
          else: return
        
        # TODO how to unselect by user?
      
        if self.selected is True and self.viz_selected is None:
            
            rospy.loginfo("object " + self.id + ": selected")
            self.pub.publish(self.id)
            
            self.viz_selected = self.scene.addEllipse(0, 0, 180, 180, QtCore.Qt.green, QtCore.Qt.green)
            self.viz_selected.setParentItem(self.viz)
            self.viz_selected.setPos(150/2 - 180/2, 150/2 - 180/2)
            self.viz_selected.setFlag(QtGui.QGraphicsItem.ItemStacksBehindParent)
            self.scene.removeItem(self.viz_preselect)
            self.viz_preselect = None

class pointing_point():
    
    def __init__(self,  id,  scene):
        
        self.id = id
        self.scene = scene
        
        self.pos = (0,  0)
        self.last_pos = (0,  0)
        self.last_timer_pos = (0,  0)
        
        self.viz = None
        self.timestamp = None
        
        self.last_move = None
        self.moving = False
        
        self.timer = QtCore.QTimer()
        self.timer.start(500)
        self.timer.timeout.connect(self.timer_evt)
    
    def is_moving(self):
        
        return self.moving
    
    def set_pos(self,  pos):
        
        self.timestamp = rospy.Time.now()
        self.last_pos = self.pos
        self.pos = pos
        #print("set_pos for :" + self.id)
        
        if self.viz is None:
        
            self.viz = self.scene.addEllipse(0, 0, 50, 50, QtCore.Qt.blue, QtCore.Qt.blue)
            self.viz.setZValue(100)
            
        self.viz.setPos(self.pos[0] - 25, self.pos[1] - 25)

    def is_active(self):
        
        if self.timestamp is None: return False
        else: return True

    def timer_evt(self):
        
        if self.timestamp is None: return
        
        if self.last_timer_pos is not None:
        
            dx = abs(self.pos[0] - self.last_timer_pos[0])
            dy = abs(self.pos[1] - self.last_timer_pos[1])
            m = min(self.scene.width(),  self.scene.height())
            
            if dx  > m*0.001 or dy > m*0.001:
                
                self.last_move = rospy.Time.now()
                
            if rospy.Time.now() - self.last_move > rospy.Duration(2):
                self.moving = False
            else: self.moving = True
        
        self.last_timer_pos = self.pos
        
        if rospy.Time.now() - self.timestamp > rospy.Duration(1):
            
            self.scene.removeItem(self.viz)
            self.viz = None
            self.timestamp = None
