from art_instructions.gui import GuiInstruction
from PyQt4 import QtCore

translate = QtCore.QCoreApplication.translate


class WaitForUser(GuiInstruction):

    CONTEXT = "WaitForUser"

    def __init__(self, ui):

        super(WaitForUser, self).__init(ui)


class WaitForUserLearn(WaitForUser):

    def __init__(self, ui):

        super(WaitForUserLearn, self).__init(ui)


class WaitForUserRun(WaitForUser):

    def __init__(self, ui):

        super(WaitForUserRun, self).__init(ui)

        self.ui.notif(translate(self.CONTEXT, "Waiting for user to finish"))
