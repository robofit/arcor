from art_instructions.gui import GuiInstruction
from PyQt4 import QtCore

translate = QtCore.QCoreApplication.translate


class WaitForUser(GuiInstruction):

    def __init__(self, *args, **kwargs):

        super(WaitForUser, self).__init__(*args, **kwargs)

        self.name = translate("WaitForUser", "Wait for user")


class WaitForUserLearn(WaitForUser):

    def __init__(self, *args, **kwargs):

        super(WaitForUserLearn, self).__init__(*args, **kwargs)


class WaitForUserRun(WaitForUser):

    def __init__(self, *args, **kwargs):

        super(WaitForUserRun, self).__init__(*args, **kwargs)

        self.ui.notif(translate("WaitForUser", "Waiting for user to finish"))
