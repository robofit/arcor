#!/usr/bin/env python

from ui_core import UICore
from PyQt4 import QtCore,  QtGui
import rospy
from art_msgs.msg import InstancesArray,  UserStatus
from fsm import FSM
from transitions import MachineError
from art_msgs.msg import ProgramItem as ProgIt
#from button_item import ButtonItem
from items import ObjectItem, ButtonItem,  PoseStampedCursorItem
from helpers import ProjectorHelper,  ArtApiHelper
from art_interface_utils.interface_state_manager import interface_state_manager
from art_msgs.msg import InterfaceState,  InterfaceStateItem
from sensor_msgs.msg import CompressedImage
import qimage2ndarray
import numpy as np
import cv2
import thread
import Queue

translate = QtCore.QCoreApplication.translate

class UICoreRos(UICore):

    def __init__(self):

        origin = rospy.get_param("~scene_origin",  [0,  0])
        size = rospy.get_param("~scene_size",  [1.2,  0.75])

        super(UICoreRos,  self).__init__(origin[0], origin[1],  size[0],  size[1])

        QtCore.QObject.connect(self, QtCore.SIGNAL('objects'), self.object_cb_evt)
        QtCore.QObject.connect(self, QtCore.SIGNAL('user_status'), self.user_status_cb_evt)
        QtCore.QObject.connect(self, QtCore.SIGNAL('interface_state'), self.interface_state_evt)

        self.user_status = None
        self.selected_program_id = None

        self.program_vis.active_item_switched = self.active_item_switched
        self.program_vis.program_state_changed = self.program_state_changed

        self.fsm = FSM()

        # TODO do this automatically??
        # map callbacks from FSM to this instance
        self.fsm.cb_start_calibration = self.cb_start_calibration
        self.fsm.cb_waiting_for_user = self.cb_waiting_for_user
        self.fsm.cb_program_selection = self.cb_program_selection
        self.fsm.cb_waiting_for_user_calibration = self.cb_waiting_for_user_calibration
        self.fsm.cb_learning = self.cb_learning
        self.fsm.is_template = self.is_template

        # TODO dodelat integraci state manageru (spis prehodit do ui_code a volat napr z add_polygon apod.)
        self.state_manager = interface_state_manager("PROJECTED UI",  cb=self.interface_state_cb)

        cursors = rospy.get_param("~cursors",  [])
        for cur in cursors:
            self.scene_items.append(PoseStampedCursorItem(self.scene,  self.rpm,  cur))

        self.scene_items.append(ButtonItem(self.scene,  self.rpm,  0,  0,  "STOP",  None,  self.stop_btn_clicked,  2.0,  QtCore.Qt.red))
        self.scene_items[-1].setPos(self.scene.width()-self.scene_items[-1].w,  self.scene.height() - self.scene_items[-1].h - 60)
        self.scene_items[-1].set_enabled(True)

        self.scene_pub = rospy.Publisher("scene",  CompressedImage,  queue_size=1,  tcp_nodelay=True,  latch=True)
        self.last_scene_update = None

        self.scene_img_deq = Queue.Queue(maxsize=1)
        thread.start_new_thread(self.scene_pub_thread,  ())

        self.scene.changed.connect(self.scene_changed)

        self.projectors = []

        projs = rospy.get_param("~projectors",  [])
        for proj in projs:
            self.add_projector(proj)

        self.art = ArtApiHelper()

    def start(self):

        rospy.loginfo("Waiting for ART services...")
        self.art.wait_for_api()

        rospy.loginfo("Waiting for projector nodes...")
        for proj in self.projectors:
            proj.wait_until_available()

        rospy.loginfo("Ready! Starting state machine.")

        # TODO move this to ArtApiHelper ??
        self.obj_sub = rospy.Subscriber('/art/object_detector/object_filtered', InstancesArray, self.object_cb, queue_size=1)
        self.user_status_sub = rospy.Subscriber('/art/user/status',  UserStatus,  self.user_status_cb,  queue_size=1)

        self.fsm.tr_start()

    def add_projector(self,  proj_id):

        self.projectors.append(ProjectorHelper(proj_id))

    def scene_pub_thread(self):

        while not rospy.is_shutdown():

            try:
                pix = self.scene_img_deq.get(block=True, timeout=1)
            except Queue.Empty:
                continue

            img = pix.toImage()
            img = img.convertToFormat(QtGui.QImage.Format_ARGB32)

            v =qimage2ndarray.rgb_view(img)

            msg = CompressedImage()
            msg.header.stamp = rospy.Time.now()
            msg.format = "png"
            msg.data = np.array(cv2.imencode('.png', v,  (cv2.cv.CV_IMWRITE_PNG_COMPRESSION, 3))[1]).tostring()
            #print len(msg.data)

            self.scene_pub.publish(msg)

    def scene_changed(self,  rects):

        if len(rects) == 0: return
        if self.scene_img_deq.full(): return

        # TODO does it make sense to limit FPS?
        now = rospy.Time.now()
        if self.last_scene_update is None:
            self.last_scene_update = now
        else:
            if now - self.last_scene_update < rospy.Duration(1.0/20):
                return

        self.last_scene_update = now

        pix = QtGui.QPixmap(self.scene.width(), self.scene.height())
        painter = QtGui.QPainter(pix)
        self.scene.render(painter)
        painter.end()

        try:
            self.scene_img_deq.put_nowait(pix)
        except Queue.Full:
            pass

    def stop_btn_clicked(self):

        # TODO
        self.notif(translate("UICoreRos", "Emergency stop pressed"),  temp=True)

    def interface_state_evt(self,  state):

        # TODO !!

        if state.current_syst_state == InterfaceStateItem.STATE_PROGRAM_RUNNING:

            if state.is_clear():

                self.clear_all()

            self.program_vis.set_running()

            # TODO check state.program_id and load it if its different from loaded one
            self.program_vis.set_active(inst_id=state.instruction_id)

    def interface_state_cb(self,  state):

        self.emit(QtCore.SIGNAL('interface_state'),  state)

    # callback from ProgramItem (button press)
    def program_state_changed(self,  state):

        if state == 'RUNNING':

            prog = self.program_vis.get_prog()
            prog.id = 1

            if not self.art.store_program(prog):
                # TODO what to do?
                self.notif(translate("UICoreRos", "Failed to store program"),  temp=True)

            else:

                self.notif(translate("UICoreRos", "Program stored. Starting..."),  temp=True)

            self.art.start_program(prog.id)
            self.fsm.tr_program_learned()

        # TODO pause / stop -> fsm
        #elif state == ''

    # callback from ProgramItem
    def active_item_switched(self):

        rospy.logdebug("Program ID:" + str(self.program_vis.prog.id) + ", active item ID: " + str(self.program_vis.active_item.item.id))

        self.clear_all()
        self.state_manager.clear_all()

        if self.program_vis.active_item.item.type in [ProgIt.MANIP_PICK,  ProgIt.MANIP_PLACE,  ProgIt.MANIP_PICK_PLACE]:

            if self.program_vis.active_item.item_learned():

                self.notif(translate("UICoreRos", "This program item seems to be done"))

            else:

                # TODO vypsat jaky je to task?
                self.notif(translate("UICoreRos", "Program current manipulation task"))

            #self.state_manager.set_syst_state(InterfaceStateItem.STATE_LEARNING,  self.program_vis.prog.id,  self.program_vis.active_item.id)

            # TODO loop across program item ids - not indices!!
            idx = self.program_vis.items.index(self.program_vis.active_item)
            if idx  > 0:
                for i in range(idx-1, -1, -1):

                    it = self.program_vis.items[i]

                    if it.item.type in  [ProgIt.MANIP_PLACE,  ProgIt.MANIP_PICK_PLACE] and it.is_place_pose_set():

                        # TODO add "artif." object instead of place??
                        self.add_place(translate("UICoreRos", "OBJECT FROM STEP") + " " + str(it.item.id),  it.item.place_pose.pose.position.x,  it.item.place_pose.pose.position.y,  fixed=True)
                        break

            if self.program_vis.active_item.item.spec == ProgIt.MANIP_TYPE:

                if not self.program_vis.active_item.is_object_set():

                    self.notif(translate("UICoreRos", "Select object type to be picked up"),  temp=True)

                self.select_object_type(self.program_vis.active_item.item.object)

                # if program item already contains polygon - let's display it
                if self.program_vis.active_item.is_pick_polygon_set():

                    self.add_polygon(translate("UICoreRos", "PICK POLYGON"),  poly_points=self.program_vis.active_item.get_pick_polygon_points(),  polygon_changed=self.polygon_changed)

            else:

                self.notif(translate("UICoreRos", "Select object to be picked up"),  temp=True)
                self.select_object(self.program_vis.active_item.item.object)

            if self.program_vis.active_item.is_object_set():

                # TODO kdy misto place pose pouzi place polygon? umoznit zmenit pose na polygon a opacne?

                if  self.program_vis.active_item.is_place_pose_set():
                    (x,  y) = self.program_vis.active_item.get_place_pose()
                    self.add_place(translate("UICoreRos", "PLACE POSE"),  x,  y,  self.place_pose_changed)
                else:
                    self.notif(translate("UICoreRos", "Set where to place picked object"))
                    self.add_place(translate("UICoreRos", "PLACE POSE"),  self.width/2,  self.height/2,  self.place_pose_changed)

    def place_pose_changed(self,  pos):

        self.program_vis.set_place_pose(pos[0],  pos[1])

    def is_template(self):

        return True

    def cb_learning(self):

        # TODO zobrazit instrukce k tasku
        pass

    def calib_done_cb(self,  proj):

        if proj.is_calibrated():

            self.calib_proj_cnt += 1

            if self.calib_proj_cnt < len(self.projectors):

                self.projectors[self.calib_proj_cnt].calibrate(self.calib_done_cb)
            else:
                rospy.loginfo('Projectors calibrated.')
                self.fsm.tr_calibrated()

        else:

            # calibration failed - let's try again
            rospy.logerror('Calibration failed for projector: ' + proj.proj_id)
            proj.calibrate(self.calib_done_cb)

    def cb_start_calibration(self):

        if len(self.projectors) == 0:

            rospy.loginfo('No projectors to calibrate.')
            self.fsm.tr_calibrated()

        else:

            rospy.loginfo('Starting calibration of ' + str(len(self.projectors)) + ' projector(s)')

            self.calib_proj_cnt = 0

            if not self.projectors[0].calibrate(self.calib_done_cb):
                # TODO what to do?
                rospy.logerror("Failed to start projector calibration")

    def cb_waiting_for_user(self):

        self.notif(translate("UICoreRos", "Waiting for user..."))

    def cb_waiting_for_user_calibration(self):

        self.notif(translate("UICoreRos", "Please do a calibration pose"))

    def cb_program_selection(self):

        self.notif(translate("UICoreRos", "Please select a program"))

        # TODO display list of programs -> let user select -> then load it
        self.program = self.art.load_program(0)

        if self.program is not None: # TODO avoid self.program -> duplication

            self.program_vis.set_prog(self.program,  self.is_template())
            self.active_item_switched()
            self.fsm.tr_program_selected()
        else:
           self.notif(translate("UICoreRos", "Loading of requested program failed"),  temp=True)

    def object_cb(self,  msg):

        self.emit(QtCore.SIGNAL('objects'),  msg)

    def object_cb_evt(self,  msg):

        for obj_id in msg.lost_objects:

            self.remove_object(obj_id)
            self.notif(translate("UICoreRos", "Object") + " ID=" + str(obj_id) + " " + translate("UICoreRos", "disappeared"), temp=True)

        for inst in msg.instances:

            obj = self.get_object(inst.object_id)

            if obj: obj.set_pos(inst.pose.position.x,  inst.pose.position.y)
            else:

                self.add_object(inst.object_id,  inst.object_type,  inst.pose.position.x,  inst.pose.position.y,  self.object_selected)
                self.notif(translate("UICoreRos", "New object") + " ID=" + str(inst.object_id),  temp=True)

    def polygon_changed(self,  pts):

        self.program_vis.set_polygon(pts)

    def object_selected(self,  id,  selected):

        if self.fsm.state != 'learning': return False

        # TODO handle un-selected

        print "selected object: " + str(id)

        obj = self.get_object(id)

        # TODO test typu operace

        if self.program_vis.active_item.item.spec == ProgIt.MANIP_TYPE:

            # this type of object is already set
            if obj.object_type == self.program_vis.active_item.item.object: return
            else:
                # TODO remove previously inserted polygon, do not insert new place
                pass

            poly_points = []

            self.program_vis.set_object(obj.object_type)
            self.select_object_type(obj.object_type)

            for obj in self.get_scene_items_by_type(ObjectItem):
                poly_points.append(obj.get_pos())

            self.add_polygon(translate("UICoreRos", "PICK POLYGON"),  poly_points,  polygon_changed=self.polygon_changed)
            self.notif(translate("UICoreRos", "Check and adjust pick polygon"),  temp=True)

            self.notif(translate("UICoreRos", "Set where to place picked object"),  temp=True)
            self.add_place(translate("UICoreRos", "PLACE POSE"),  self.width/2,  self.height/2,  self.place_pose_changed)

        elif self.program_vis.active_item.item.spec == ProgIt.MANIP_ID:

            # TODO
            pass

        return True


    def user_status_cb(self,  msg):

        self.emit(QtCore.SIGNAL('user_status'),  msg)

    def user_status_cb_evt(self,  msg):

        self.user_status = msg

        try:

            if self.user_status.user_state == UserStatus.USER_NOT_CALIBRATED:

                self.fsm.tr_user_present()

            elif self.user_status.user_state == UserStatus.USER_CALIBRATED:

                self.fsm.tr_user_calibrated()

        except MachineError:
            pass
