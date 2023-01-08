from constants.robot_type import RobotType

# TODO delte this one and move info about
# task to robot class

ROBOT_USECASE_MAP = {
    RobotType.MOBILE: ['GT'],
    RobotType.MANIPULATOR: ['MT']
}