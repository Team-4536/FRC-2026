import wpilib

from mb.controls import Ctrlr
from mb.LEDSignals import LEDSignals
from mb.subsystem import Subsystem
from mb.swerveDrive import SwerveDrive
from mb.utils import TimeData
from ntcore import NetworkTableInstance
from typing import List


class Robot(wpilib.TimedRobot):
    def robotInit(self) -> None:
        self.table: NetworkTableInstance = NetworkTableInstance.getDefault().getTable("telemetry")

        self.driveCtrlr: Ctrlr = Ctrlr(0)
        self.mechCtrlr: Ctrlr = Ctrlr(1)
        self.buttonPanel = wpilib.Joystick(4)

        self.ledSignals: LEDSignals = LEDSignals(0)
        self.swerveDrive: SwerveDrive = SwerveDrive()
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

    def disabledInit(self) -> None:
        self.disabledPeriodic()

    def disabledPeriodic(self) -> None:
        for s in self.subsystems:
            s.disable()


if __name__ == "__main__":
    wpilib.run(Robot)
