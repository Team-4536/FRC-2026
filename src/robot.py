import wpilib

from mb.drive import SwerveDrive
from mb.inputs import Inputs
from mb.LEDSignals import LEDSignals
from mb.subsystem import Subsystem
from mb.utils import TimeData
from ntcore import NetworkTableInstance
from typing import List


class Robot(wpilib.TimedRobot):
    def robotInit(self) -> None:
        self.table: NetworkTableInstance = NetworkTableInstance.getDefault().getTable("telemetry")

        self.inputs: Inputs = Inputs()
        self.swerveDrive: SwerveDrive = SwerveDrive.symmetricDrive(
            frontLeftDriveDev=2,
            frontLeftAzimuthDev=1,
            frontLeftEncoderDev=21,
            frontRightDriveDev=4,
            frontRightAzimuthDev=3,
            frontRightEncoderDev=22,
            backLeftDriveDev=6,
            backLeftAzimuthDev=5,
            backLeftEncoderDev=23,
            backRightDriveDev=8,
            backRightAzimuthDev=7,
            backRightEncoderDev=24,
            driveGearing=6.12,
            azimuthGearing=21.4,
            xPos=1,
            yPos=1,
        )
        self.ledSignals: LEDSignals = LEDSignals(deviceId=0)
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
