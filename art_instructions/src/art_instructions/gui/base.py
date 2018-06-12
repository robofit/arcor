from PyQt4 import QtCore
import rospy

translate = QtCore.QCoreApplication.translate


class GuiInstruction(QtCore.QObject):

    NAME = ""  # localized name of the instruction

    def __init__(self, ui, block_id, item_id, editable=False, flags=None):

        super(GuiInstruction, self).__init__()

        self.ui = ui
        self.block_id = block_id
        self.instruction_id = item_id
        self.editable = editable
        self.flags = flags

    def logerr(self, msg):

        rospy.logerr(self.NAME + ": " + msg)

    def logwarn(self, msg):

        rospy.logwarn(self.NAME + ": " + msg)

    def logdebug(self, msg):

        rospy.logdebug(self.NAME + ": " + msg)

    @staticmethod
    def get_text(ph, block_id, item_id):
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
    def cid(self):
        """Current instruction id"""

        return self.block_id, self.instruction_id

    def learning_done(self):
        """Called when user presses 'Done' button."""

        return

    def detected_objects(self, msg):

        return
