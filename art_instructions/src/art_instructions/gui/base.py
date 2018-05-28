from PyQt4 import QtCore


class GuiInstruction(QtCore.QObject):

    def __init__(self, ui):

        self.ui = ui

    def cleanup(self):

        raise NotImplemented()

    def object_selected(self, object_id, selected):

        raise NotImplemented()

# TODO class/plugin for ProgramItem ??
