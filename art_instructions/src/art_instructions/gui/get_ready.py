from art_instructions.gui import GuiInstruction
from PyQt4 import QtCore

translate = QtCore.QCoreApplication.translate


class GetReady(GuiInstruction):

    CONTEXT = "GetReady"

    def __init__(self, *args, **kwargs):

        super(GetReady, self).__init__(*args, **kwargs)


class GetReadyLearn(GetReady):

    def __init__(self, *args, **kwargs):

        super(GetReadyLearn, self).__init__(*args, **kwargs)


class GetReadyRun(GetReady):

    def __init__(self, *args, **kwargs):

        super(GetReadyRun, self).__init__(*args, **kwargs)

        self.ui.notif(translate(self.CONTEXT, "Robot is getting ready"))
