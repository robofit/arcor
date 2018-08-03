from PyQt4 import QtCore

translate = QtCore.QCoreApplication.translate

# TODO define/implement "hooks"


class GuiPlugin(QtCore.QObject):

    BASE_NS = "/art/interface/projected_gui/"

    def __init__(self, ui):

        super(GuiPlugin, self).__init__()

        self.ui = ui

    def m2pix(self, m):

        return self.ui.scene.rpm*m

    def init(self):
        """If plugin needs to wait for some other API, it should do it here."""
        pass

    def notify_info(self):
        pass

    def notify_warn(self):
        pass

    def visualize(self):

        pass
