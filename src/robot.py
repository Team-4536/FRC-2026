from drive import SwerveDrive
from inputs import Inputs
from LEDSignals import LEDSignals
from ntcore import NetworkTableInstance
from rev import ClosedLoopConfig, ClosedLoopSlot, MAXMotionConfig, SparkMaxConfig
from subsystem import Subsystem
from typing import List
from utils import TimeData
from wpilib import TimedRobot


class Robot(TimedRobot):
    def robotInit(self) -> None:
        self.table: NetworkTableInstance = NetworkTableInstance.getDefault().getTable("telemetry")

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
            xPos=1,  # DEFINETLY INCORRECT
            yPos=1,
        )
        self.ledSignals: LEDSignals = LEDSignals(deviceId=0)
        self.time: TimeData = TimeData()

        self.swerveDrive.configureDriveMotors(config=self.configDrive)
        self.swerveDrive.configureAzimuthMotors(config=self.configAzimuth)

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

    @property
    def configDrive(self):
        return (
            SparkMaxConfig()
            .smartCurrentLimit(40)
            .disableFollowerMode()
            .setIdleMode(SparkMaxConfig.IdleMode.kBrake)
            .apply(
                ClosedLoopConfig()
                .pidf(0.00019, 0, 0, 0.00002)
                .setFeedbackSensor(ClosedLoopConfig.FeedbackSensor.kPrimaryEncoder)
                .outputRange(-1.0, 1.0, ClosedLoopSlot.kSlot0)
                .apply(
                    MAXMotionConfig()
                    .maxVelocity(2000, ClosedLoopSlot.kSlot0)
                    .maxAcceleration(50000, ClosedLoopSlot.kSlot0)
                    .allowedClosedLoopError(1)
                )
            )
        )

    @property
    def configAzimuth(self):
        return (
            SparkMaxConfig()
            .smartCurrentLimit(40)
            .inverted(True)
            .setIdleMode(SparkMaxConfig.IdleMode.kBrake)
            .apply(
                ClosedLoopConfig()
                .pidf(0.15, 0, 0, 0)
                .setFeedbackSensor(ClosedLoopConfig.FeedbackSensor.kPrimaryEncoder)
                .outputRange(-1.0, 1.0, ClosedLoopSlot.kSlot0)
                .positionWrappingEnabled(False)
                .apply(
                    MAXMotionConfig()
                    .maxVelocity(5000, ClosedLoopSlot.kSlot0)
                    .maxAcceleration(10000, ClosedLoopSlot.kSlot0)
                    .allowedClosedLoopError(0.2)
                )
            )
        )


if __name__ == "__main__":
    wpilib.run(Robot)
