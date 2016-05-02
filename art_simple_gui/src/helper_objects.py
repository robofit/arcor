import rospy
from PyQt4 import QtGui, QtCore
from geometry_msgs.msg import PoseStamped
import numpy as np

class scene_place():
    
    def __init__(self,  scene, pos,  pub, wsize, h_matrix):
        
        self.scene = scene
        self.pos = pos
        self.pub = pub
        self.viz = self.scene.addEllipse(0, 0, 100, 100, QtCore.Qt.cyan, QtCore.Qt.cyan)
        self.viz.setPos(self.pos[0] - 100/2, self.pos[1] - 100/2)
        self.wsize = wsize
        self.h_matrix = h_matrix
        
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
        
        p = np.array([[px], [py], [1.0]])
        res = np.linalg.inv(self.h_matrix)*p

        ps.pose.position.x = float(res[0]/res[2])
        ps.pose.position.y = float(res[1]/res[2])
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
        #self.viz.setPos(self.pos[0] + 150/2, self.pos[1] + 150/2)
        #self.viz.setPos(self.pos[0], self.pos[1])
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
        
        if self.viz not in pointing_obj.collidingItems(): return False
        
        self.pointed_at = rospy.Time.now()
        
        if not self.preselected:
            
            rospy.loginfo("object " + self.id + ": preselected")
            
            self.preselected = True
            self.preselected_at = rospy.Time.now()
        
            self.viz_preselect = self.scene.addEllipse(0, 0, 180, 180, QtCore.Qt.gray, QtCore.Qt.gray)
            self.viz_preselect.setParentItem(self.viz)
            self.viz_preselect.setPos(150/2 - 180/2, 150/2 - 180/2)
            self.viz_preselect.setFlag(QtGui.QGraphicsItem.ItemStacksBehindParent)
            
            return False
        
        if not self.selected:
           
          if rospy.Time.now() - self.preselected_at > rospy.Duration(1): self.selected = True
          else: return False
        
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
            return True

class pointing_point():
    
    def __init__(self,  id,  scene):
        
        self.id = id
        self.scene = scene
        
        self.pos = (0,  0)
        
        self.viz = None
        self.timestamp = None
        
        self.xyt = []
        self.pointed_pos = None
        
        self.timer = QtCore.QTimer()
        self.timer.start(100)
        self.timer.timeout.connect(self.timer_evt)

    def get_pointed_place(self):
        
        return self.pointed_pos
    
    def set_pos(self,  pos):
        
        self.timestamp = rospy.Time.now()
        self.pos = pos
        
        # TODO what to do if pos differs much? like from 1280 to 0 or so
        
        self.xyt.append([self.pos, rospy.Time.now()])
        
        if self.viz is None:
        
            rospy.loginfo("Enabling pointing-point: " + self.id)
            self.viz = self.scene.addEllipse(0, 0, 50, 50, QtCore.Qt.blue, QtCore.Qt.blue)
            self.viz.setZValue(100)
            
        self.viz.setPos(self.pos[0] - 25, self.pos[1] - 25)

    def is_active(self):
        
        if self.timestamp is None: return False
        else: return True

    def timer_evt(self):
        
        if self.timestamp is None: return
        
        now = rospy.Time.now()
        
        # throw away older data
        while len(self.xyt) > 0 and now - self.xyt[0][1] > rospy.Duration(2):
            
            self.xyt.pop(0)
        
        # wait until we have some data
        if len(self.xyt) > 10 and now - self.xyt[0][1] > rospy.Duration(1.5):
            
            x = []
            y = []
            
            for it in self.xyt:
                
                x.append(it[0][0])
                y.append(it[0][1])
        
            xm = np.mean(x)
            ym = np.mean(y)
            
            xs = np.std(x)
            ys = np.std(y)
            
            # if "cursor" position move a bit (noise) but doesn't move too much - the user is pointing somewhere
            if xs > 0.01 and xs < 15.0 and ys > 0.01 and ys < 15.0:
            #if xs < 10.0 and ys < 10.0: # -> this is only for testing
                
                self.pointed_pos = (int(xm),  int(ym))
                
            else:
                
                self.pointed_pos = None
        
        if len(self.xyt) == 0:
            
            rospy.loginfo("Disabling pointing-point: " + self.id)
            self.pointed_pos = None
            self.scene.removeItem(self.viz)
            self.viz = None
            self.timestamp = None
