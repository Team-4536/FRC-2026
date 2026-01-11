from drive import SwerveDrive
from inputs import Inputs
from LEDSignals import LEDSignals
from motor import RevMotor
from ntcore import NetworkTable, NetworkTableInstance
from subsystem import Subsystem
from typing import List
from utils import TimeData
from wpilib import run, TimedRobot


class Robot(TimedRobot):
    def robotInit(self) -> None:
        self.table: NetworkTable = NetworkTableInstance.getDefault().getTable("telemetry")

        self.inputs: Inputs = Inputs()
        self.swerveDrive: SwerveDrive = SwerveDrive.symmetricDrive(
            frontLeftDriveID=2,
            frontLeftAzimuthID=1,
            frontRightDriveID=4,
            frontRightAzimuthID=3,
            backLeftDriveID=6,
            backLeftAzimuthID=5,
            backRightDriveID=8,
            backRightAzimuthID=7,
            driveGearing=6.12,
            azimuthGearing=21.4,
            xPos=1,
            yPos=1,
        )
        self.ledSignals: LEDSignals = LEDSignals(deviceID=0)
        self.time: TimeData = TimeData()

        self.swerveDrive.configureDriveMotors(config=RevMotor.driveConfig)
        self.swerveDrive.configureAzimuthMotors(config=RevMotor.azimuthConfig)

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
        self.inputs.periodic()

        self.swerveDrive.periodic(self.inputs.fieldSpeeds)

    def disabledInit(self) -> None:
        self.disabledPeriodic()

    def disabledPeriodic(self) -> None:
        for s in self.subsystems:
            s.disable()


if __name__ == "__main__":
    run(Robot)  # type: ignore
