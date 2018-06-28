from PyQt4 import QtCore

translate = QtCore.QCoreApplication.translate


class GuiPlugin(QtCore.QObject):

    def __init__(self, ui):

        super(GuiPlugin, self).__init__()

        self.ui = ui

    def visualize(self):

        pass
