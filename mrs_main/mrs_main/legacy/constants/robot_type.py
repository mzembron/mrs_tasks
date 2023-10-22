from enum import Enum

# TODO delte this one and move info about
# task to robot class

class RobotType(Enum):
    MOBILE = "mobile"
    MANIPULATOR = "manipulator"
    HYBRID = "hybrid"
