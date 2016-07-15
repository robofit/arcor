import rospy
from art_msgs.msg import InterfaceState
from geometry_msgs.msg import PoseStamped,  PointStamped

class interface_state_manager():

    def __init__(self, int_type,  cb = None,  event_filter=[],  interface_filter=[]):
    
        self.int_type = int_type
        self.interface_state_pub = rospy.Publisher("/art/interface/state", InterfaceState, queue_size=10)
        self.interface_state_sub = rospy.Subscriber('/art/interface/state', InterfaceState, self.state)
        self.cb = cb
        self.event_filter = event_filter
        self.interface_filter = interface_filter
    
    def publish(self,  event,  data=None):
        
        if self.int_type == InterfaceState.INT_LISTENER: return
        
        ints = InterfaceState()
        ints.interface_type=self.int_type
        ints.event_type = event
        
        # TODO option: remember events and then send them periodically ??
        
        if data is not None and not isinstance(data,  list): data = [data]
        
        if event == InterfaceState.EVT_OBJECT_ID_SELECTED or event == InterfaceState.EVT_OBJECT_TYPE_SELECTED:

            if not all(isinstance(n, basestring) for n in data): raise('data should be string!')
            ints.data = data
        
        elif event == InterfaceState.EVT_PLACE_SELECTED:
            
            if not all(isinstance(n, PoseStamped) for n in data): raise('data should be PoseStamped!')
            ints.places = data
            
        elif event == InterfaceState.EVT_POLYGON:
            
            if not all(isinstance(n, PointStamped) for n in data): raise('data should be PointStamped!')
            if (len(data)) < 3: raise('Polygon should have 3 points at least!')
            ints.points = data
            
        elif event == InterfaceState.EVT_STATE_PROGRAM_RUNNING or event == InterfaceState.EVT_STATE_LEARNING:

            if not all(isinstance(n, int) for n in data): raise('data should be int!')
            if len(data) != 2: raise('data should contain program ID and step ID!')
            ints.idata = data
            
        elif event == InterfaceState.EVT_PROGRAM_TEMPLATE_SELECTED:
            
            if not all(isinstance(n, int) for n in data): raise('data should be int!')
            ints.idata = data
        
        self.interface_state_pub.publish(ints)

    def state(self,  msg):
        
        # we don't want to hear our own messages or messages from listeners (those should be ignored)
        if (msg.interface_type == self.int_type or msg.interface_type == InterfaceState.INT_LISTENER): return
        
        # filter events
        if len(self.event_filter) > 0 and msg.event_type not in self.event_filter: return
        if len(self.interface_filter) > 0 and msg.interface_type not in self.interface_filter: return

        if self.cb is not None: self.cb(msg)
