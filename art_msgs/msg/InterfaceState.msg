string BRAIN_ID=ART_BRAIN

# interfaces sends "events" - diffs to a current state to /art/interface/events
# brain publishes current world state on a latched topic: /art/interface/state
bool is_diff
string interface_id # should be unique for each interface and BRAIN_ID for the brain

InterfaceStateItem[] items
