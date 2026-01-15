import rev
import wpilib


class RobotHAL:
    def __init__(self):

        self.motor = rev.SparkMax(
            1, rev.SparkMax.MotorType.kBrushless
        )  # set the ID to an actual value
