from art_instructions.gui import GuiInstruction
from PyQt4 import QtCore

translate = QtCore.QCoreApplication.translate


class WaitForUser(GuiInstruction):

    CONTEXT = "WaitForUser"

    def __init__(self, *args, **kwargs):

        super(WaitForUser, self).__init__(*args, **kwargs)


class WaitForUserLearn(WaitForUser):

    def __init__(self, *args, **kwargs):

        super(WaitForUserLearn, self).__init__(*args, **kwargs)


class WaitForUserRun(WaitForUser):

    def __init__(self, *args, **kwargs):

        super(WaitForUserRun, self).__init__(*args, **kwargs)

        self.ui.notif(translate(self.CONTEXT, "Waiting for user to finish"))
