class BrainInstruction(object):

    def __init__(self, brain):

        self.brain = brain

    def cleanup(self):

        pass

    @property
    def block_id(self):

        return self.brain.block_id

    @property
    def state_manager(self):

        return self.brain.state_manager

    @property
    def ph(self):

        return self.brain.ph

    @property
    def instruction(self):

        return self.brain.instruction


class BrainFSM(object):

    def __init__(self, brain):

        self.brain = brain
        self.fsm = brain.fsm

