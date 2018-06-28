from art_instructions.gui import GuiInstruction
from PyQt4 import QtCore

translate = QtCore.QCoreApplication.translate


class WaitUntilUserFinishes(GuiInstruction):

    NAME = translate("WaitUntilUserFinishes", "Wait until user finishes")

    def __init__(self, *args, **kwargs):

        super(WaitUntilUserFinishes, self).__init__(*args, **kwargs)


class WaitUntilUserFinishesLearn(WaitUntilUserFinishes):

    def __init__(self, *args, **kwargs):

        super(WaitUntilUserFinishesLearn, self).__init__(*args, **kwargs)


class WaitUntilUserFinishesRun(WaitUntilUserFinishes):

    def __init__(self, *args, **kwargs):

        super(WaitUntilUserFinishesRun, self).__init__(*args, **kwargs)

        self.ui.notif(translate("WaitUntilUserFinishes", "Waiting for user"))
