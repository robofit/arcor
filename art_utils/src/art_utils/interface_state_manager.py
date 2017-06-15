import rospy
from art_msgs.msg import InterfaceState, KeyValue


class InterfaceStateManager(object):

    def __init__(self, interface_id, cb=None):

        self.brain = False
        self.state = InterfaceState()
        self.interface_id = interface_id
        self.cb = cb

        if self.interface_id == InterfaceState.BRAIN_ID:

            rospy.loginfo('InterfaceStateManager: brain mode')
            self.brain = True
            self.interface_state_pub = rospy.Publisher("/art/interface/state", InterfaceState, queue_size=10, latch=True)
            self.interface_state_sub = rospy.Subscriber('/art/interface/state_evt', InterfaceState, self.state_cb)

        else:

            rospy.loginfo('InterfaceStateManager: interface mode')
            self.interface_state_pub = rospy.Publisher("/art/interface/state_evt", InterfaceState, queue_size=10)
            self.interface_state_sub = rospy.Subscriber('/art/interface/state', InterfaceState, self.state_cb)

    def state_cb(self, msg):

        if msg.interface_id == self.interface_id:
            return

        flags = {}

        for kv in msg.flags:

            flags[kv.key] = kv.value

        # TODO use thread
        if self.cb is not None:
            self.cb(self.state, msg, flags)

        if self.interface_id != InterfaceState.BRAIN_ID:

            self.state = msg

        else:

            # don't want to modify system_state, error_severity, error_code

            self.state.timestamp = msg.timestamp
            self.state.program_id = msg.program_id
            self.state.block_id = msg.block_id
            self.state.program_current_item = msg.program_current_item
            self.state.flags = msg.flags

            self.send()

    def get_system_state(self):

        return self.state.system_state

    def set_system_state(self, st, auto_send=True):

        self.state.timestamp = rospy.Time.now()
        self.state.system_state = st

        if auto_send:
            self.send()

    def update_program_item(self, prog_id, block_id, program_item_msg, flags={}, auto_send=True):

        self.state.timestamp = rospy.Time.now()
        self.state.program_id = prog_id
        self.state.block_id = block_id
        self.state.program_current_item = program_item_msg
        self.state.flags = []

        for (k, v) in flags.iteritems():

            self.state.flags.append(KeyValue(k, v))

        if auto_send:
            self.send()

    def send(self):

        # TODO send only if auto_send was not used
        self.state.interface_id = self.interface_id
        self.interface_state_pub.publish(self.state)

    def set_error(self, error_severity, error_code, auto_send=True):

        self.state.timestamp = rospy.Time.now()
        self.state.error_severity = error_severity
        self.state.error_code = error_code

        if auto_send:
            self.send()
