#!/usr/bin/env python

from art_msgs.msg import InterfaceState
from PyQt4 import QtCore

translate = QtCore.QCoreApplication.translate

error_dict = {
    InterfaceState.ERROR_UNKNOWN: translate("ErrorStrings", "Unknown error"),
    InterfaceState.ERROR_OBJECT_MISSING: translate("ErrorStrings", "Cannot find object"),
    InterfaceState.ERROR_OBJECT_MISSING_IN_POLYGON: translate("ErrorStrings", "There is no object left in the polygon"),
    InterfaceState.ERROR_NO_GRIPPER_AVAILABLE: translate("ErrorStrings", "No gripper available"),
    InterfaceState.ERROR_OBJECT_IN_GRIPPER: translate("ErrorStrings", "Robot already holds object and cannot grasp another"),
    InterfaceState.ERROR_NO_OBJECT_IN_GRIPPER: translate("ErrorStrings", "Robot should hold object but it doesn't."),
    InterfaceState.ERROR_PICK_FAILED: translate("ErrorStrings", "Robot failed to pick the object."),
    InterfaceState.ERROR_PLACE_FAILED: translate("ErrorStrings", "Robot failed to place the object."),
}


def get_error_string(error):

    if error not in error_dict:

        return "Undefined error"

    return error_dict[error]
