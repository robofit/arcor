from art_helpers.instructions_helper import InstructionsHelper, InstructionsHelperException
from art_helpers.program_helper import ProgramHelper, ProgramHelperException
from art_helpers.art_robot_helper import ArtRobotArmHelper, ArtRobotHelper, UnknownRobot,\
    RobotParametersNotOnParameterServer
from art_helpers.interface_state_manager import InterfaceStateManager
from art_helpers.calibration_helper import ArtCalibrationHelper

import rospy


def array_from_param(param, target_type=str, expected_length=None):

    tmp = []

    for s in rospy.get_param(param).split(","):
        tmp.append(target_type(s.strip()))

    if expected_length is not None:
        assert len(tmp) == expected_length

    return tmp
