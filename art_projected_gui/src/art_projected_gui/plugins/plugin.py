from PyQt4 import QtCore

translate = QtCore.QCoreApplication.translate

# TODO define/implement "hooks"


class GuiPlugin(QtCore.QObject):

    def __init__(self, ui):

        super(GuiPlugin, self).__init__()

        self.ui = ui

    def m2pix(self, m):

        return self.ui.scene.rpm*m

    def visualize(self):

        pass
