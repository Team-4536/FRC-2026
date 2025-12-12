import wpilib

from mb.inputs import Inputs
from mb.LEDSignals import LEDSignals
from mb.subsystem import Subsystem
from mb.swerveDrive import SwerveDrive
from mb.utils import TimeData
from ntcore import NetworkTableInstance
from typing import List


class Robot(wpilib.TimedRobot):
    def robotInit(self) -> None:
        self.table: NetworkTableInstance = NetworkTableInstance.getDefault().getTable("telemetry")

        self.inputs: Inputs = Inputs()
        self.swerveDrive: SwerveDrive = SwerveDrive()
        self.ledSignals: LEDSignals = LEDSignals()
        self.time: TimeData = TimeData()

        self.subsystems: List[Subsystem] = [
            self.ledSignals,
            self.swerveDrive,
            self.time,
        ]

    def robotPeriodic(self) -> None:
        self.time.periodic()

    def teleopInit(self) -> None:
        for s in self.subsystems:
            s.init()

    def teleopPeriodic(self) -> None:
        self.ledSignals.periodic()
        self.swerveDrive.periodic()
        self.inputs.periodic()

        self.swerveDrive.drive(self.inputs.fieldSpeeds)

    def disabledInit(self) -> None:
        self.disabledPeriodic()

    def disabledPeriodic(self) -> None:
        for s in self.subsystems:
            s.disable()


if __name__ == "__main__":
    wpilib.run(Robot)
