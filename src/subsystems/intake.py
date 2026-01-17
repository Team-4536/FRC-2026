from rev import SparkMax
from subsystems.subsystem import Subsystem
from enum import Enum
from inputs import Inputs
import wpilib
from desiredState import DesiredState


class Intake(Subsystem):
    def __init__(self):
        # Change from a 9
        self.intakeMotor = SparkMax(9, SparkMax.MotorType.kBrushless)
        self.intakeVoltage = 0

    def periodic(self, ds):
        pass

    def update(
        self,
    ):
        if DesiredState.AButton == True:
            self.intakeVoltage = 1
        else:
            self.intakeVoltage = 0

        self.intakeMotor.setVoltage(self.intakeVoltage)
