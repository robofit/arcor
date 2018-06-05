from art_instructions.gui import GuiInstruction
from PyQt4 import QtCore

translate = QtCore.QCoreApplication.translate


class WaitUntilUserFinishes(GuiInstruction):

    def __init__(self, *args, **kwargs):

        super(WaitUntilUserFinishes, self).__init__(*args, **kwargs)

        self.name = translate("WaitUntilUserFinishes", "Wait until user finishes")


class WaitUntilUserFinishesLearn(WaitUntilUserFinishes):

    def __init__(self, *args, **kwargs):

        super(WaitUntilUserFinishesLearn, self).__init__(*args, **kwargs)


class WaitUntilUserFinishesRun(WaitUntilUserFinishes):

    def __init__(self, *args, **kwargs):

        super(WaitUntilUserFinishesRun, self).__init__(*args, **kwargs)

        self.ui.notif(translate("WaitUntilUserFinishes", "Waiting for user"))
