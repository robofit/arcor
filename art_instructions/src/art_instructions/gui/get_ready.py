from art_instructions.gui import GuiInstruction
from PyQt4 import QtCore

translate = QtCore.QCoreApplication.translate


class GetReady(GuiInstruction):

    CONTEXT = "GetReady"

    def __init__(self, ui):

        super(GetReady, self).__init(ui)


class GetReadyLearn(GetReady):

    def __init__(self, ui):

        super(GetReadyLearn, self).__init(ui)


class GetReadyRun(GetReady):

    def __init__(self, ui):

        super(GetReadyRun, self).__init(ui)

        self.ui.notif(translate(self.CONTEXT, "Robot is getting ready"))
