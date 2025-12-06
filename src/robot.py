import math
import wpilib

from LEDSignals import LEDSignals
from ntcore import NetworkTableInstance
from subsystem import Subsystem
from swerveDrive import SwerveDrive
from typing import List
from utils import CircularScalar, Scalar, TimeData, lerp, wrapAngle
from controls import Ctrlr


class Robot(wpilib.TimedRobot):
    def robotInit(self) -> None:
        self.table: NetworkTableInstance = NetworkTableInstance.getDefault().getTable("telemetry")

        self.driveCtrlr: Ctrlr = Ctrlr(0)
        self.mechCtrlr: Ctrlr = Ctrlr(1)
        self.buttonPanel = wpilib.Joystick(4)

        self.ledSignals: LEDSignals = LEDSignals(0)
        self.swerveDrive: SwerveDrive = SwerveDrive()
        self.time: TimeData = TimeData()
        self.linScalar: Scalar = Scalar()
        self.cirScalar: CircularScalar = CircularScalar()

        self.state: int = 0

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
        self.state = (
            1
            if self.driveCtrlr.getAButtonPressed()
            else (
                2
                if self.driveCtrlr.getBButtonPressed()
                else 3 if self.driveCtrlr.getYButtonPressed() else self.state
            )
        )

        self.ledSignals.periodic()
        self.swerveDrive.periodic(
            # *self.cirScalar(self.driveCtrlr.getLeftX(), -self.driveCtrlr.getLeftY()),
            # self.linScalar(self.driveCtrlr.getRightX()),
        )

    def disabledInit(self) -> None:
        self.disabledPeriodic()

    def disabledPeriodic(self) -> None:
        for s in self.subsystems:
            s.disable()


if __name__ == "__main__":
    wpilib.run(Robot)
