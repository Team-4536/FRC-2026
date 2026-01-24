from rev import SparkMax
from subsystems.subsystem import Subsystem
from enum import Enum
from inputs import Inputs
import wpilib
from desiredState import DesiredState


class Intake(Subsystem):
    def __init__(self):
        # Change from a 9
        self.intakeMotor = SparkMax(999999999, SparkMax.MotorType.kBrushless)
        self.intakeVoltage = 0

    def periodic(self):
        pass

    def update(self, ds: DesiredState):
        if ds.AButton():
            self.intakeVoltage = 1
        else:
            self.intakeVoltage = 0

        self.intakeMotor.setVoltage(self.intakeVoltage)
