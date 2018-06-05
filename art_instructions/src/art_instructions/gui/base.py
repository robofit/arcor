from PyQt4 import QtCore

translate = QtCore.QCoreApplication.translate


class GuiInstruction(QtCore.QObject):

    def __init__(self, ui, editable=False, flags=None):

        super(GuiInstruction, self).__init__()

        self.ui = ui
        self.editable = editable
        self.flags = flags
        self.name = ""  # localized name of the instruction

    def get_text(self):
        """
        Returns additional (localized) text displayed below instruction name in program visualization
        """

        return ""

    def cleanup(self):
        """Removes all previously created scene items (such as dialogs)."""

        pass

    def object_selected(self, obj, selected, msg):

        return

    @property
    def block_id(self):

        # TODO raise exception if...
        return self.ui.program_vis.block_id

    @property
    def instruction_id(self):
        # TODO raise exception if...
        return self.ui.program_vis.get_current_item().id

    @property
    def cid(self):
        """Current instruction id"""

        return self.block_id, self.instruction_id

    def learning_done(self):
        """Called when user presses 'Done' button."""

        return

    def detected_objects(self, msg):

        return
