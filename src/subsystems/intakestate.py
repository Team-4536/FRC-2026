from enum import Enum


class IntakeState(Enum):
    # OH_NO = 0
    UP = 1
    GOING_DOWN = 2
    DOWN = 3
    GOING_UP = 4
    # when using states for autos, do IntakeState.[insert state], it will act as a simple check,
    # if needed I can make a more robust system
