from art_instructions.gui import GuiInstruction
from PyQt4 import QtCore

translate = QtCore.QCoreApplication.translate


class WaitUntilUserFinishes(GuiInstruction):

    CONTEXT = "WaitUntilUserFinishes"

    def __init__(self, ui):

        super(WaitUntilUserFinishes, self).__init(ui)


class WaitUntilUserFinishesLearn(WaitUntilUserFinishes):

    def __init__(self, ui):

        super(WaitUntilUserFinishesLearn, self).__init(ui)


class WaitUntilUserFinishesRun(WaitUntilUserFinishes):

    def __init__(self, ui):

        super(WaitUntilUserFinishesRun, self).__init(ui)

        self.ui.notif(translate(self.CONTEXT, "Waiting for user"))
