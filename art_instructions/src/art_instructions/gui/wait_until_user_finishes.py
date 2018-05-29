from art_instructions.gui import GuiInstruction
from PyQt4 import QtCore

translate = QtCore.QCoreApplication.translate


class WaitUntilUserFinishes(GuiInstruction):

    CONTEXT = "WaitUntilUserFinishes"

    def __init__(self, *args, **kwargs):

        super(WaitUntilUserFinishes, self).__init__(*args, **kwargs)


class WaitUntilUserFinishesLearn(WaitUntilUserFinishes):

    def __init__(self, *args, **kwargs):

        super(WaitUntilUserFinishesLearn, self).__init__(*args, **kwargs)


class WaitUntilUserFinishesRun(WaitUntilUserFinishes):

    def __init__(self, *args, **kwargs):

        super(WaitUntilUserFinishesRun, self).__init__(*args, **kwargs)

        self.ui.notif(translate(self.CONTEXT, "Waiting for user"))
