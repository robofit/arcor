from PyQt4 import QtCore

# TODO method to get string for program_vis


class GuiInstruction(QtCore.QObject):

    def __init__(self, ui, editable=False, flags=None):

        self.ui = ui

    def cleanup(self):

        raise NotImplemented()

    def object_selected(self, obj, selected, msg):

        return

    @property
    def block_id(self):

        # TODO raise exception if...
        return self.ui.program_vis.block_id

    @property
    def instruction_id(self):
        # TODO raise exception if...
        return self.program_vis.get_current_item().id

    @property
    def cid(self):
        """Current instruction id"""

        return self.block_id, self.instruction_id

    def learning_done(self):

        return

    def detected_objects(self, msg):

        return

# TODO class/plugin for ProgramItem ??
