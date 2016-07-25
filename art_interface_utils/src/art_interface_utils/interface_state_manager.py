import rospy
from art_msgs.msg import InterfaceState
from geometry_msgs.msg import PoseStamped,  PointStamped,  PolygonStamped
from std_srvs.srv import Empty, EmptyResponse

class interface_state_manager():

    def __init__(self, int_type,  cb = None,  event_filter=[],  interface_filter=[],  aggregate_full_state = False):
    
        self.aggregate_full_state = aggregate_full_state
        self.int_type = int_type
        self.interface_state_pub = rospy.Publisher("/art/interface/state", InterfaceState, queue_size=10)
        self.interface_state_sub = rospy.Subscriber('/art/interface/state', InterfaceState, self.state)
        self.cb = cb
        self.event_filter = event_filter
        self.interface_filter = interface_filter
        
        if self.int_type == InterfaceState.INT_BRAIN:
            
             # TODO test if service already exists? There should be only one brain...
            
            self.aggregate_full_state = True
            self.srv_full_update = rospy.Service('/art/interface/full_update', Empty, self.full_update_cb)
        
        if self.aggregate_full_state:
            
            self.state = None
            self.state_data = []
            self.clear_all()
    
    def clear_all(self):
        
        self.selected_obj_id = []
        self.selected_obj_type = []
        self.selected_places = []
        self.selected_polygons = []
    
    def full_update(self):
    
        if self.state is not None: self.publish(self.state,  self.state_data)
        if len(self.selected_obj_id) > 0: self.publish(InterfaceState.EVT_OBJECT_ID_SELECTED,  self.selected_obj_id)
        if len(self.selected_obj_type) > 0: self.publish(InterfaceState.EVT_OBJECT_TYPE_SELECTED,  self.selected_obj_type)
        if len(self.selected_polygons) > 0: self.publish(InterfaceState.EVT_POLYGON,  self.selected_polygons)
    
    def full_update_cb(self,  req):
        
        self.full_update()
        return EmptyResponse()
    
    def publish(self,  event,  data=None):
        
        if self.int_type == InterfaceState.INT_LISTENER: return
        
        ints = InterfaceState()
        ints.interface_type=self.int_type
        ints.event_type = event
        
        # TODO option: remember events and then send them periodically ??
        
        if data is not None and not isinstance(data,  list) and not isinstance(data,  tuple): data = [data]
        
        if event == InterfaceState.EVT_OBJECT_ID_SELECTED or event == InterfaceState.EVT_OBJECT_TYPE_SELECTED:
            
            if data is None or not all(isinstance(n, basestring) for n in data): raise ValueError('data should be string!')
            ints.data = data
        
        elif event == InterfaceState.EVT_PLACE_SELECTED:
            
            if data is None or not all(isinstance(n, PoseStamped) for n in data): raise ValueError('data should be PoseStamped!')
            ints.places = data
            
        elif event == InterfaceState.EVT_POLYGON:
            
            if data is None or not all(isinstance(n, PolygonStamped) for n in data): raise ValueError('data should be PolygonStamped!')
            for poly in data:
                if (len(poly.polygon.points)) < 3: raise ValueError('Polygon should have 3 points at least!')
            ints.polygons = data
            
        elif event == InterfaceState.EVT_STATE_PROGRAM_RUNNING or event == InterfaceState.EVT_STATE_LEARNING:

            if data is None or not all(isinstance(n, int) for n in data): raise ValueError('data should be int!')
            if len(data) != 2: raise ValueError('data should contain program ID and step ID!')
            ints.idata = data
            
        elif event == InterfaceState.EVT_PROGRAM_TEMPLATE_SELECTED:
            
            if data is None or not all(isinstance(n, int) for n in data): raise ValueError('data should be int!')
            ints.idata = data
        
        # events with no data
        elif event in [InterfaceState.EVT_INT_NOT_READY,  InterfaceState.EVT_INT_ENABLED,  InterfaceState.EVT_INT_DISABLED,  InterfaceState.EVT_INT_READY,  InterfaceState.EVT_STATE_PROGRAM_STOPPED,  InterfaceState.EVT_STATE_PROGRAM_FINISHED,  InterfaceState.EVT_CLEAR_ALL]:
            
            pass
            
        else:
            
            raise NotImplementedError('Event type ' + str(event) + ' is not yet implemented :(')
        
        self.interface_state_pub.publish(ints)

    def state(self,  msg):
        
        # full state should contain even our own events
        if self.aggregate_full_state:
                
            if msg.event_type == InterfaceState.EVT_OBJECT_ID_SELECTED:
                
                for obj in msg.data:
                    
                    if obj not in self.selected_obj_id:
                        
                        self.selected_obj_id.append(obj)
                        
            elif msg.event_type == InterfaceState.EVT_OBJECT_TYPE_SELECTED:
                
                for obj in msg.data:
                    
                    if obj not in self.selected_obj_type:
                        
                        self.selected_obj_type.append(obj)
                        
            elif msg.event_type == InterfaceState.EVT_CLEAR_ALL:
                
                self.clear_all()
                
            elif msg.event_type in [InterfaceState.EVT_STATE_LEARNING,  InterfaceState.EVT_STATE_PROGRAM_RUNNING,  InterfaceState.EVT_STATE_PROGRAM_STOPPED,  InterfaceState.EVT_STATE_PROGRAM_FINISHED]:
                
                self.state = msg.event_type
                self.state_data = msg.idata
                
            elif msg.event_type == InterfaceState.EVT_POLYGON:
                
                self.selected_polygons = msg.polygons
                
            # elif # TODO object/place unselected
                
            else: 
            
                rospy.logerr('Unknown event type: ' + str(msg.event_type))

        # we don't want to hear our own messages or messages from listeners (those should be ignored)
        if (msg.interface_type == self.int_type or msg.interface_type == InterfaceState.INT_LISTENER): return

        # filter events
        if len(self.event_filter) > 0 and msg.event_type not in self.event_filter: return
        if len(self.interface_filter) > 0 and msg.interface_type not in self.interface_filter: return

        if self.cb is not None: self.cb(msg)
