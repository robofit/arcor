import rospy
from art_msgs.msg import InterfaceState,  InterfaceStateItem
from geometry_msgs.msg import PoseStamped,  PointStamped,  PolygonStamped
from std_srvs.srv import Empty, EmptyResponse

# TODO state: methods for adding objects/places etc. ?
# helper class - holding current state of the 'scene'
class state():
    
    def __init__(self):
        
        self.changed = False
        
        self.selected_object_ids = set()
        self.selected_object_types = set()
        self.selected_places = []
        self.polygons = []
        
        self.syst_states = [InterfaceStateItem.STATE_UNKNOWN, InterfaceStateItem.STATE_INITIALIZING,  InterfaceStateItem.STATE_IDLE,  InterfaceStateItem.STATE_LEARNING,  InterfaceStateItem.STATE_PROGRAM_RUNNING,  InterfaceStateItem.STATE_PROGRAM_STOPPED,  InterfaceStateItem.STATE_PROGRAM_FINISHED]
        
        self.current_syst_state = InterfaceStateItem.STATE_UNKNOWN
        
        self.program_id = None
        self.instruction_id = None
        
    def is_clear(self):
        
        return (len(self.selected_object_ids) + len(self.selected_object_types) + len(self.selected_places) + len(self.polygons)) == 0
        
    def clear_all(self):
        
        if not self.is_clear(): self.changed = True
        
        self.selected_object_ids.clear()
        self.selected_object_types.clear()
        self.selected_places = []
        self.polygons = []
        
    def is_place_selected(self,  place):
        
        for p in self.selected_places:
            
            # we want to ignore timestamp
            # TODO is this good idea?
            if p.pose == place.pose: return True
            
        return False
            
    def is_polygon_selected(self,  polygon):
        
        for p in self.polygons:
            
            # TODO is this good idea?
            if p.polygon == polygon.polygon: return True
                
        return False

class interface_state_manager():
    
    def __init__(self,  interface_id,  cb = None):
        
        self.interface_id = interface_id
        
        self.brain = False
        self.state = state()
        self.cb = cb
        
        if self.interface_id == InterfaceState.BRAIN_ID:
            
            rospy.loginfo('interface_state_manager: brain mode')
            
            self.brain = True
            self.interface_state_pub = rospy.Publisher("/art/interface/state", InterfaceState, queue_size=1,  latch = True)
            
            # full state will not be published after each event, but max at given rate
            self.timer = rospy.Timer(rospy.Duration(1), self.timer_cb)
            
        else:
            
            rospy.loginfo('interface_state_manager: interface mode')
            
            self.interface_state_sub = rospy.Subscriber('/art/interface/state', InterfaceState, self.state_cb) # published by brain
        
        self.interface_event_pub = rospy.Publisher("/art/interface/events", InterfaceState, queue_size=100)
        self.interface_event_sub = rospy.Subscriber('/art/interface/events', InterfaceState, self.event_cb) # published by other interfaces
    
    def timer_cb(self,  event):
        
        if self.state.changed is True:
            
            self.state.changed = False
            self.__publish_state()
        
    def __publish_state(self):
        
        msg = InterfaceState()
        msg.is_diff = False
        msg.interface_id = self.interface_id
        
        sys_state = InterfaceStateItem()
        sys_state.type = self.state.current_syst_state
        
        if self.state.current_syst_state in [InterfaceStateItem.STATE_LEARNING,  InterfaceStateItem.STATE_PROGRAM_STOPPED,  InterfaceStateItem.STATE_PROGRAM_RUNNING]:
                    
            sys_state.idata.append(self.state.program_id)
            sys_state.idata.append(self.state.instruction_id)
                    
        elif self.state.current_syst_state == InterfaceStateItem.STATE_PROGRAM_FINISHED:
            
            sys_state.idata.append(self.state.program_id)
            
        msg.items.append(sys_state)
        
        for obj_id in self.state.selected_object_ids:
            
            tmp = InterfaceStateItem()
            tmp.type = InterfaceStateItem.SELECTED_OBJECT_ID
            tmp.data.append(obj_id)
            msg.items.append(tmp)
            
        for obj_type in self.state.selected_object_types:
            
            tmp = InterfaceStateItem()
            tmp.type = InterfaceStateItem.SELECTED_OBJECT_TYPE
            tmp.data.append(obj_type)
            msg.items.append(tmp)
            
        for place in self.state.selected_places:
            
            tmp = InterfaceStateItem()
            tmp.type = InterfaceStateItem.SELECTED_PLACE
            tmp.place = place
            msg.items.append(tmp)
            
        for polygon in self.state.polygons:
            
            tmp = InterfaceStateItem()
            tmp.type = InterfaceStateItem.POLYGON
            tmp.polygon = polygon
            msg.items.append(tmp)
        
        self.interface_state_pub.publish(msg)
    
    def __get_msg(self):
        
        msg = InterfaceState()
        msg.is_diff = True
        msg.interface_id = self.interface_id
        msg.items.append(InterfaceStateItem())
        return msg
        
    def __publish_msg(self,  msg):
        
        self.interface_event_pub.publish(msg)
    
    def set_syst_state(self,  syst_state,  prog_id=None,  inst_id=None):
        
        if syst_state not in self.state.syst_states: raise ValueError('Unknown syst_state')
        if prog_id is not None and not isinstance(prog_id,  int): raise ValueError('prog_id should be int')
        if inst_id is not None and not isinstance(inst_id,  int): raise ValueError('inst_id should be int')
        
        if syst_state in [InterfaceStateItem.STATE_LEARNING,  InterfaceStateItem.STATE_PROGRAM_STOPPED,  InterfaceStateItem.STATE_PROGRAM_RUNNING] and (prog_id is None or inst_id is None): raise ValueError('prog_id and inst_id should be set')
        elif syst_state == InterfaceStateItem.STATE_PROGRAM_FINISHED and prog_id is None: raise ValueError('prog_id should be set')
        
        if self.state.current_syst_state == syst_state and self.state.program_id == prog_id and self.state.instruction_id == inst_id: return
        
        self.state.current_syst_state = syst_state
        self.state.program_id = prog_id
        self.state.instruction_id = inst_id
        self.state.changed = True
            
        msg = self.__get_msg()
        msg.items[0].type = syst_state
        if prog_id is not None: msg.items[0].idata.append(prog_id)
        if inst_id is not None: msg.items[0].idata.append(inst_id)
        self.__publish_msg(msg)
    
    def select_object_id(self,  obj_id):
        
        if not isinstance(obj_id, basestring): raise ValueError('obj_id should be string!')
        
        if obj_id in self.state.selected_object_ids: return
        
        self.state.selected_object_ids.add(obj_id)
        self.state.changed = True
        
        msg = self.__get_msg()
        msg.items[0].type = InterfaceStateItem.SELECTED_OBJECT_ID
        msg.items[0].data.append(obj_id)
        self.__publish_msg(msg)
    
    def unselect_object_id(self,  obj_id):
        
        raise NotImplementedError('To be implemented...')
        
    def select_object_type(self,  obj_type):
        
        if not isinstance(obj_type, basestring): raise ValueError('obj_type should be string!')
        
        if obj_type in self.state.selected_object_types: return
        
        self.state.selected_object_types.add(obj_type)
        self.state.changed = True
        
        msg = self.__get_msg()
        msg.items[0].type = InterfaceStateItem.SELECTED_OBJECT_TYPE
        msg.items[0].data.append(obj_type)
        self.__publish_msg(msg)
        
    def unselect_object_type(self,  obj_type):
        
        raise NotImplementedError('To be implemented...')
        
    def select_place(self,  place):
        
        if not isinstance(place, PoseStamped): raise ValueError('place should be PoseStamped!')
        
        if self.state.is_place_selected(place): return
        
        self.state.selected_places.append(place)
        self.state.changed = True
        
        msg = self.__get_msg()
        msg.items[0].type = InterfaceStateItem.SELECTED_PLACE
        msg.items[0].place = place
        self.__publish_msg(msg)
        
    def select_polygon(self,  poly):
        
        if not isinstance(poly, PolygonStamped): raise ValueError('poly should be PolygonStamped!')
        
        if self.state.is_polygon_selected(poly): return
        
        self.state.polygons.append(poly)
        self.state.changed = True
        
        msg = self.__get_msg()
        msg.items[0].type = InterfaceStateItem.POLYGON
        msg.items[0].polygon = poly
        self.__publish_msg(msg)
        
    def unselect_place(self,  place):
        
        raise NotImplementedError('To be implemented...')
        
    def int_state(self,  ready):
        
        # TODO save to state? disable for brain?
        
        msg = self.__get_msg()
        if ready:  msg.items[0].type = InterfaceStateItem.INT_READY
        else: msg.items[0].type = InterfaceStateItem.INT_NOT_READY
        self.__publish_msg(msg)
        
    # TODO enabled / disabled
        
    def clear_all(self):
        
        if self.state.is_clear(): return
        
        self.state.clear_all()
        
        msg = self.__get_msg()
        msg.items[0].type = InterfaceStateItem.CLEAR_ALL
        self.__publish_msg(msg)
    
    def __proc_item(self,  it):
        
        change = False
        
        if it.type in self.state.syst_states:
                
                if self.state.current_syst_state != it.type: change = True
                self.state.current_syst_state = it.type
                
                if it.type in [InterfaceStateItem.STATE_LEARNING,  InterfaceStateItem.STATE_PROGRAM_STOPPED,  InterfaceStateItem.STATE_PROGRAM_RUNNING]:
                    
                    if self.state.program_id != it.idata[0]: change = True
                    if self.state.instruction_id != it.idata[1]: change = True
                    
                    self.state.program_id = it.idata[0]
                    self.state.instruction_id = it.idata[1]
                    
                elif it.type == InterfaceStateItem.STATE_PROGRAM_FINISHED:
                    
                    self.state.program_id = it.idata[0]
                    
        elif it.type == InterfaceStateItem.SELECTED_OBJECT_ID:
            
            if it.data[0] not in self.state.selected_object_ids: change = True
            self.state.selected_object_ids.add(it.data[0])
            
        elif it.type == InterfaceStateItem.SELECTED_OBJECT_TYPE:
            
            if it.data[0] not in self.state.selected_object_types: change = True
            self.state.selected_object_types.add(it.data[0])
            
        elif it.type == InterfaceStateItem.POLYGON:
            
            if not self.state.is_polygon_selected(it.polygon):
            
                change = True
                self.state.polygons.append(it.polygon)
            
        elif it.type == InterfaceStateItem.SELECTED_PLACE:
            
            if not self.state.is_place_selected(it.place):
        
                change = True
                self.state.selected_places.append(it.place)
            
        elif it.type ==  InterfaceStateItem.CLEAR_ALL:
            
            self.state.clear_all()
            
        else:
            
            rospy.logwarn('type ' + str(it.type) + 'not yet implemented')
            
        if change: self.state.changed = True
        return change
    
    def event_cb(self,  msg):
        
        # ignore our own messages and non diff ones
        if msg.interface_id == self.interface_id or not msg.is_diff: return
        
        # diff message should have exactly one item
        if len(msg.items) != 1:
                
                rospy.logerr('Ignoring diff message with ' + str(len(msg.items)) + ' items!')
                return
                
        if self.__proc_item(msg.items[0]) and self.cb is not None: self.cb(self.state)
    
    def state_cb(self,  msg):
        
        # full state should be sent only by brain... no diffs allowed here
        if msg.interface_id != InterfaceState.BRAIN_ID or msg.is_diff: return

        for it in msg.items:
            
            self.__proc_item(it)
                
        if self.cb is not None and self.state.changed:
            
            self.state.changed = False
            self.cb(self.state)
