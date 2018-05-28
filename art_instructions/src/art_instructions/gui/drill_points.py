from art_instructions.gui import GuiInstruction


class DrillPoints(GuiInstruction):

    def __init__(self, ui):

        super(DrillPoints, self).__init(ui)

    def cleanup(self):

        super(DrillPoints, self).cleanup()


class DrillPointsLearn(DrillPoints):

    def __init__(self, ui, editable=False):

        super(DrillPointsLearn, self).__init(ui)

        self.editable = editable


class PickFromFeederRun(DrillPoints):

    def __init__(self, ui):

        super(DrillPointsLearn, self).__init(ui)
