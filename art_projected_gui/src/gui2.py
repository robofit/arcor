#!/usr/bin/env python

import rospy
import rospkg
import sys
import signal

from std_msgs.msg import String, Bool,  UInt8
from PyQt4 import QtGui, QtCore, QtOpenGL
from art_msgs.msg import InstancesArray,  UserStatus,  InterfaceStateItem
from art_msgs.srv import getProgram,  storeProgram,  startProgram
from art_msgs.msg import ProgramItem
from geometry_msgs.msg import Pose,  PoseStamped, PointStamped,  PolygonStamped
from std_srvs.srv import Empty, EmptyResponse
import actionlib

from helper_objects import scene_place,  scene_object,  pointing_point,  scene_polygon,  dist
from gui_calibration import gui_calibration
from program_widget import program_widget
from art_interface_utils.interface_state_manager import interface_state_manager
import numpy as np

# TODO keep place shown (otherwise it's hard to select polygon)
# TODO set self.program_started based on InterfaceState
# TODO stop program (button?)
# TODO "smart" label - able to show messages for defined time, more messages at the time, images (?) etc.
# TODO move program_widget automatically so it does not collide with objects etc.
# TODO make 180deg rotation configurable

def sigint_handler(*args):
    """Handler for the SIGINT signal."""
    sys.stderr.write('\r')
    QtGui.QApplication.quit()

class simple_gui(QtGui.QWidget):

    def __init__(self):
       
        super(simple_gui, self).__init__()

        self.tfl = None

        self.inited = False

        rospack = rospkg.RosPack()
        self.img_path = rospack.get_path('art_projected_gui') + '/imgs'

        self.obj_sub = rospy.Subscriber('/art/object_detector/object_filtered', InstancesArray, self.object_cb, queue_size=1)
        self.point_left_sub = rospy.Subscriber('/art/user/pointing_left', PoseStamped, self.pointing_point_left_cb,  queue_size=1)
        self.point_right_sub = rospy.Subscriber('/art/user/pointing_right', PoseStamped, self.pointing_point_right_cb,  queue_size=1)
        self.user_status_sub = rospy.Subscriber('/art/user/status',  UserStatus,  self.user_status_cb,  queue_size=1)

        self.enabled_int = True
        
        self.srv_enable_int = rospy.Service('~enable', Empty, self.enable_int)
        self.srv_disable_int = rospy.Service('~disable', Empty, self.disable_int)
        
        self.srv_show_marker = rospy.Service('~show_marker', Empty, self.show_marker)
        self.srv_hide_marker = rospy.Service('~hide_marker', Empty, self.hide_marker)

        self.viz_objects = {} # objects can be accessed by their ID
        self.viz_places = [] # array of scene_place objects
        self.viz_polygons = [] # array of scene_polygon objects

        # these are used to know progress of task item programming
        self.object_selected = False
        self.place_selected = False
        self.polygon_selected = False
        
        self.selected_at = None  # timestamp of last learned program item

        self.scene=QtGui.QGraphicsScene(self)
        self.scene.setBackgroundBrush(QtCore.Qt.black)
        self.view = QtGui.QGraphicsView(self.scene, self)
        self.view.setRenderHint(QtGui.QPainter.Antialiasing)
        self.view.setViewportUpdateMode(QtGui.QGraphicsView.FullViewportUpdate)
        self.view.setStyleSheet( "QGraphicsView { border-style: none; }" )
        #self.view.setViewport(QtOpenGL.QGLWidget()) # rendering using OpenGL -> somehow broken :(

        self.pm = QtGui.QPixmap(self.img_path + "/koberec.png") # TODO use homography matrix to correct it
        self.marker = self.scene.addPixmap(self.pm.scaled(self.size(), QtCore.Qt.KeepAspectRatio))
        self.marker.setZValue(-100)
        self.marker.hide()

        self.resizeEvent = self.on_resize

        self.pointing_left = pointing_point("left", self.scene)
        self.pointing_right = pointing_point("right", self.scene)
        self.pointing_mouse = pointing_point("mouse", self.scene,  True)

        QtCore.QObject.connect(self, QtCore.SIGNAL('objects'), self.objects_evt)
        QtCore.QObject.connect(self, QtCore.SIGNAL('pointing_point_left'), self.pointing_point_left_evt)
        QtCore.QObject.connect(self, QtCore.SIGNAL('pointing_point_right'), self.pointing_point_right_evt)
        QtCore.QObject.connect(self, QtCore.SIGNAL('clear_all()'), self.clear_all_evt)
        QtCore.QObject.connect(self, QtCore.SIGNAL('show_marker()'), self.show_marker_evt)
        QtCore.QObject.connect(self, QtCore.SIGNAL('hide_marker()'), self.hide_marker_evt)
        QtCore.QObject.connect(self, QtCore.SIGNAL('user_status'), self.user_status_evt)
        
        self.timer = QtCore.QTimer()
        self.timer.start(500)
        self.timer.timeout.connect(self.timer_evt)

        self.label = self.scene.addText("Waiting for user",  QtGui.QFont('Arial', 26))
        self.label.rotate(180)
        self.label.setDefaultTextColor(QtCore.Qt.white)
        self.label.setZValue(200)

        self.user_status = None

        self.calib = gui_calibration(self.scene,  self.img_path,  self.width())
        self.calib.on_request = self.on_calib_req
        self.calib.on_finished = self.on_calib_finished
        
        self.ignored_items = [self.label,  self.marker, self.calib.checkerboard] # items in the scene to be ignored when checking for "collisions"
        
        self.prog = program_widget(self)
        self.prog.resize(500, 200)
        self.prog.move(10, 10)
        self.prog.show()
    
        self.srv_brain_start_program = rospy.ServiceProxy('/art/brain/program/start', startProgram)
        
        # TODO only for testing - program should be selected by user
        self.load_program(0)
        
        # we will save learned program under different ID
        self.prog.prog.id = 1 # TODO don't have it hardcoded
        
        self.program_started = False
        self.item_to_learn = None
        
        self.state_manager = interface_state_manager("PROJECTED UI",  cb=self.interface_state_cb)
        self.state_manager.int_state(False)
        
        self.inited = True
    
    def enable_int(self, req):
        
        rospy.loginfo('Projected GUI: enabling')
        self.enabled_int = True
        #self.state_manager.publish(InterfaceState.EVT_INT_ENABLED)
        return EmptyResponse()
        
    def disable_int(self,  req):
        
        rospy.loginfo('Projected GUI: disabling')
        self.enabled_int = False
        #self.state_manager.publish(InterfaceState.EVT_INT_DISABLED)
        return EmptyResponse()
    
    def on_calib_req(self):
        
        self.state_manager.int_state(False)
        self.prog.hide()
        
    def on_calib_finished(self):
        
        self.state_manager.int_state(True)
        self.prog.show()
    
    def interface_state_cb(self,  state):
        
            if state.current_syst_state == InterfaceStateItem.STATE_PROGRAM_RUNNING:
                
                self.program_started = True
                self.prog.set_current(state.program_id,  state.instruction_id)
            
            # TODO other states!!!
            #elif state.current_syst_state == InterfaceStateItem.STATE_LEARNING:
                
            #    pass
            
            if state.is_clear(): self.clear_all_evt()
            
            # selected object IDs
            for obj_id,  obj in self.viz_objects.iteritems():
                
                if obj_id in state.selected_object_ids:
                    
                    obj.set_selected()
                    
                elif obj.obj_type not in state.selected_object_types:
                    
                    obj.unselect()
            
            # TODO selected object types
            for obj_id,  obj in self.viz_objects.iteritems():
                
                if obj.obj_type in state.selected_object_types:
                   
                   obj.set_selected()
                    
                elif obj_id not in state.selected_object_ids:
                    
                    obj.unselect()
            
            # TODO check if polygon already exists
            # TODO remove polygons
            for poly in state.polygons:
                
                self.viz_polygons.append(scene_polygon(self.scene,  self.calib,  poly.polygon.points))
 
 
            # TODO places
        
    def start_program(self):
        
        try:
            resp = self.srv_brain_start_program(1) # TODO don't have it hardcoded ;)
        except rospy.ServiceException, e:
            print "Service call failed: %s"%e
            # TODO what to do?
        
    def load_program(self,  prog_id,  template = False):
    
        rospy.loginfo('Loading program: ' + str(prog_id))
    
        try:
            prog_srv = rospy.ServiceProxy('/art/db/program/get', getProgram)
            resp = prog_srv(prog_id)
            self.prog.set_prog(resp.program,  True)
        except rospy.ServiceException, e:
            print "Service call failed: %s"%e

    def store_program(self):
        
        prog_srv = rospy.ServiceProxy('/art/db/program/store', storeProgram)
        
        try:
            resp = prog_srv(self.prog.prog)
        except rospy.ServiceException, e:
            print "Service call failed: %s"%e
        

    def eventFilter(self, source, event):
        
        if event.type() == QtCore.QEvent.MouseMove or event.type() == QtCore.QEvent.MouseButtonPress:
            
            click = event.buttons() != QtCore.Qt.NoButton
            self.pointing_mouse.set_pos((event.pos().x(),  event.pos().y()),  click)
            self.pointing_point(self.pointing_mouse,  click)
                
        return QtGui.QMainWindow.eventFilter(self, source, event)
    
    def user_status_cb(self,  msg):
        
        self.emit(QtCore.SIGNAL('user_status'),  msg)
    
    def user_status_evt(self,  msg):
    
        self.user_status = msg
        
        if self.user_status.header.stamp == rospy.Time(0):
            self.user_status.header.stamp = rospy.Time.now()

    def show_marker(self, req):
        
        self.emit(QtCore.SIGNAL('show_marker()'))
        return EmptyResponse()
        
    def show_marker_evt(self):
        
        self.marker.show()
        
    def hide_marker(self, req):
        
        self.emit(QtCore.SIGNAL('hide_marker()'))
        return EmptyResponse()
    
    def hide_marker_evt(self):
        
        self.marker.hide()
    
    def clear_all_evt(self):
        
        self.state_manager.clear_all()
        
        self.object_selected = False
        self.place_selected = False
        #self.label.setPlainText("Waiting for user")
    
        for k,  v in self.viz_objects.iteritems():
            
            v.unselect()
    
        for it in self.viz_places:
            it.remove()
        self.viz_places = []
        
        for it in self.viz_polygons:
            it.remove()
        self.viz_polygons = []
       
    def timer_evt(self):
        
        if not self.calib.is_calibrated():

            self.label.setPlainText('Waiting for calibration...')
            return
        
        if self.user_status is not None and rospy.Time.now() - self.user_status.header.stamp > rospy.Duration(2):
            
            self.user_status = None
        
        if self.user_status is None:
            
            self.label.setPlainText('Waiting for user tracking...')
    
        elif self.user_status.user_state == UserStatus.NO_USER:
            
            self.label.setPlainText('Waiting for user...')
            
        elif self.user_status.user_state == UserStatus.USER_NOT_CALIBRATED:
            
            self.label.setPlainText('Please make a calibration pose')
            
        elif (self.user_status.user_state == UserStatus.USER_CALIBRATED):
         
           if self.program_started:

                self.label.setPlainText("Program is running...")
                return  
          
           if not (self.pointing_left.is_active() or self.pointing_right.is_active() or self.pointing_mouse.is_active()) and self.selected_at is None:
            
                self.label.setPlainText('Point at objects or places to select them')
    
    def on_resize(self, event):
    
        self.view.setFixedSize(self.width(), self.height())
        self.view.setSceneRect(QtCore.QRectF(0, 0, self.width(), self.height()))
        self.view.setHorizontalScrollBarPolicy(QtCore.Qt.ScrollBarAlwaysOff);
        self.view.setVerticalScrollBarPolicy(QtCore.Qt.ScrollBarAlwaysOff);
        self.marker.setPixmap(self.pm.scaled(self.size(), QtCore.Qt.KeepAspectRatio))
        self.label.setPos(self.width() - 70,  70)
        self.calib.resize(self.size())
      
    def object_cb(self, msg):

        if not self.inited: return
        if not self.calib.is_calibrated():

            return

        self.emit(QtCore.SIGNAL('objects'),  msg)
    
    def pointing_point_left_evt(self,  pos):

        if (pos[0] in range(0, self.width()) and pos[1] in range(0, self.height())):
           self.pointing_left.set_pos(pos)
           self.pointing_point(self.pointing_left)
           
    def pointing_point_right_evt(self,  pos):
        
        if (pos[0] in range(0, self.width()) and pos[1] in range(0, self.height())):
           
           self.pointing_right.set_pos(pos)
           self.pointing_point(self.pointing_right)
    
    def select_object(self,  pt,  click):
        
        if self.object_selected: return
        
        if self.item_to_learn.spec == ProgramItem.MANIP_ID:
            self.label.setPlainText("Select an object")
        elif self.item_to_learn.spec == ProgramItem.MANIP_TYPE:
            self.label.setPlainText("Select an object type")
            
        if not pt.is_active() or pt.viz is None: return
        
        if not self.object_selected:
            
            for k, v in self.viz_objects.iteritems():
                
                if v.pointing(pt.viz,  click) is True:
                    
                     # TODO "attach" object shape to the pointing point(s)?
                    self.object_selected = True
                    self.selected_at = rospy.Time.now()
                    
                    if self.item_to_learn.spec == ProgramItem.MANIP_ID:
                        rospy.loginfo("Object ID (" + v.id +") selected, now select place")
                        self.item_to_learn.object = v.id
                        self.state_manager.select_object_id(v.id)
                    elif self.item_to_learn.spec == ProgramItem.MANIP_TYPE:
                        rospy.loginfo("Object type (" + v.obj_type +") selected, now select polygon")
                        self.item_to_learn.object = v.obj_type
                        self.state_manager.select_object_type(v.obj_type)
                        
                        # mark all objects of this type as selected
                        for k1, v1 in self.viz_objects.iteritems():
                            if v1.obj_type == v.obj_type:
                                v1.set_selected()
                            
                    break
    
    def select_polygon(self,  pt,  click):
        
        if self.polygon_selected: return
        
        lp = len(self.viz_polygons)
        if lp==1: self.label.setPlainText("Select a pick polygon")
        elif lp==2: self.label.setPlainText("Select a place polygon")
        
        self.viz_polygons[-1].set_pos(pt.pos)
        
        pointed_place = pt.get_pointed_place()
        
        if pointed_place is not None:
            
            if len(self.viz_polygons[-1].points) > 0:
            
                d = dist(self.viz_polygons[-1].points[-1],  pointed_place)

                if d < 60: return
            
            if self.viz_polygons[-1].add_point(pointed_place):
                
                self.state_manager.select_polygon(self.viz_polygons[-1].msg())
                self.label.setPlainText("Polygon selected")
                self.polygon_selected = True
                self.selected_at = rospy.Time.now()
    
    def select_place(self,  pt,  click):
        
        if self.place_selected: return
        
        self.label.setPlainText("Select a place")
            
        items = self.scene.collidingItems(pt.viz)
        
        for iit in self.ignored_items: # TODO test it
            
            if iit in items: items.remove(iit)
        
        pointed_place = pt.get_pointed_place()
        
        if len(items) == 0 and (pointed_place is not None):
            
            # TODO how to keep some minimal spacing between places?
            skip = False
            for pl in self.viz_places:
                
                if dist(pl.pos,  pointed_place) < 150:
                    
                    rospy.logwarn(pt.id + ": place near to x=" + str(pointed_place[0]) + ", y=" + str(pointed_place[1]) + " already exists")
                    skip = True
                    break

            if skip: return

            self.label.setPlainText("Place selected at x=" + str(round(pointed_place[0],  2)) + ", y=" + str(round(pointed_place[1],  2)))
            sp = scene_place(self.scene,  pointed_place, self.size(), self.calib)
            self.state_manager.select_place(sp.get_pose())
            self.viz_places.append(sp)
            self.place_selected = True
            self.selected_at = rospy.Time.now()
            return sp
    
    def pointing_point(self,  pt,  click = False):
        
        if self.selected_at is not None:
            
            if rospy.Time.now() - self.selected_at < rospy.Duration(3): 
                
                pt.disable()
                return
            self.selected_at = None
        
        if self.program_started:
            return
        
        if self.user_status is None or self.user_status.user_state != UserStatus.USER_CALIBRATED:
            return
        
        if self.item_to_learn is None:
            self.item_to_learn = self.prog.get_item_to_learn()
            
            if self.item_to_learn is not None:
                
                self.state_manager.set_syst_state(InterfaceStateItem.STATE_LEARNING,  self.prog.prog.id,  self.item_to_learn.id)
            
            self.object_selected = False
            self.place_selected = False
            self.polygon_selected = False
        
        # everything is already learned - let's start program execution
        if self.item_to_learn is None:
            self.program_started = True
            self.start_program()
            return
        
        if self.item_to_learn.type == ProgramItem.MANIP_PICK_PLACE:
            
            if self.place_selected:
                
                self.item_to_learn = None
                self.emit(QtCore.SIGNAL('clear_all()'))
                return
            
            if not self.object_selected:
            
                self.select_object(pt,  click)
                return
            
            if self.item_to_learn.spec == ProgramItem.MANIP_TYPE:
                
                if self.polygon_selected is False:
                
                    if len(self.viz_polygons) == 0: self.viz_polygons.append(scene_polygon(self.scene,  self.calib,  []))
                    self.select_polygon(pt,  click)
                    return
            
            sp = self.select_place(pt,  click)
            
            if self.place_selected:
            
                self.item_to_learn.place_pose = sp.get_pose()
                self.item_to_learn.pick_polygon = self.viz_polygons[0].msg()
                self.label.setPlainText("Step learned")
                self.prog.learned(self.item_to_learn.id)
                self.store_program()
                
        else:
            
            # TODO other types
            pass
        
        
    def objects_evt(self,  msg):
    
        for obj_id in msg.lost_objects:
            
            try:
                self.viz_objects[obj_id].remove()
                del self.viz_objects[obj_id]
            except KeyError:
                pass
    
        for obj in msg.instances:
            
            (px, py) = self.calib.get_px(obj.pose)
            
            if obj.object_id in msg.new_objects or obj.object_id not in self.viz_objects:
                
                sobj = scene_object(self.scene,  obj.object_id, obj.object_type,   (px,  py))
                self.viz_objects[obj.object_id] = sobj
            
            else:
                
                self.viz_objects[obj.object_id].set_pos((px,  py))
           
        self.update()
     
    def pointing_point_left_cb(self, msg):
        
        if not self.inited: return
        if not self.calib.is_calibrated(): return
        if not self.enabled_int: return
       
        pos = self.calib.get_px(msg.pose)
        self.emit(QtCore.SIGNAL('pointing_point_left'),  pos)
           
    def pointing_point_right_cb(self, msg):
        
        if not self.inited: return
        if not self.calib.is_calibrated(): return
        if not self.enabled_int: return
       
        pos = self.calib.get_px(msg.pose)
        self.emit(QtCore.SIGNAL('pointing_point_right'),  pos)
       
def main(args):
    
    rospy.init_node('simple_gui', anonymous=True)
    
    rospy.loginfo('Waiting for art_brain services...')
    rospy.wait_for_service('/art/brain/program/start')
    rospy.wait_for_service('/art/brain/program/stop')
    rospy.loginfo('Waiting for art_db services...')
    rospy.wait_for_service('/art/db/program/get')
    rospy.wait_for_service('/art/db/program/store')
    rospy.loginfo('Ready!')
    
    signal.signal(signal.SIGINT, sigint_handler)

    app = QtGui.QApplication(sys.argv)
    window = simple_gui()
    
    desktop = QtGui.QDesktopWidget()
    geometry = desktop.screenGeometry(1) # 1
    window.move(geometry.left(), geometry.top())
    window.resize(geometry.width(), geometry.height())
    window.showFullScreen()
    app.installEventFilter(window)
    
    timer = QtCore.QTimer()
    timer.start(500)
    timer.timeout.connect(lambda: None)  # Let the interpreter run each 500 ms.

    sys.exit(app.exec_())
    
if __name__ == '__main__':
    try:
        main(sys.argv)
    except KeyboardInterrupt:
        print("Shutting down")
