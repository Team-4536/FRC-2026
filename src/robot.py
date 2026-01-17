from subsystemManager import SubsystemManager
from subsystems.inputs import Inputs
from subsystems.LEDSignals import LEDSignals
from subsystems.motor import RevMotor
from subsystems.swerveDrive import SwerveDrive
from subsystems.utils import TimeData
from wpilib import TimedRobot


class Robot(TimedRobot):
    def init(self) -> None:
        self.subsystems.swerveDrive.configureDriveMotors(config=RevMotor.driveConfig)
        self.subsystems.swerveDrive.configureAzimuthMotors(
            config=RevMotor.azimuthConfig
        )

    def robotInit(self) -> None:
        self.subsystems: SubsystemManager = SubsystemManager(
            inputs=Inputs(),
            ledSignals=LEDSignals(deviceID=0),
            swerveDrive=SwerveDrive.symmetricDrive(
                frontLeftDriveID=2,
                frontRightDriveID=4,
                backLeftDriveID=6,
                backRightDriveID=8,
                frontLeftAzimuthID=1,
                frontRightAzimuthID=3,
                backLeftAzimuthID=5,
                backRightAzimuthID=7,
                frontLeftEncoderID=21,
                frontRightEncoderID=22,
                backLeftEncoderID=23,
                backRightEncoderID=24,
                driveGearing=6.12,
                azimuthGearing=21.4,
                xPos=1,
                yPos=1,
            ),
            time=TimeData(),
        )

        self.init()

    def robotPeriodic(self) -> None:
        self.subsystems.time.periodic(self.subsystems.desiredState)

    def autonomousInit(self) -> None:
        self.init()

    def autonomousPeriodic(self) -> None:
        pass

    def teleopInit(self) -> None:
        self.init()

        for s in self.subsystems:
            s.init()

    def teleopPeriodic(self) -> None:
        self.subsystems.periodic()

    def teleopExit(self) -> None:
        self.disabledInit()

    def disabledInit(self) -> None:
        self.disabledPeriodic()
        self.subsystems.swerveDrive.configureDriveMotors(
            config=RevMotor.driveDisabledConfig
        )
        self.subsystems.swerveDrive.configureAzimuthMotors(
            config=RevMotor.azimuthDisabledConfig
        )

    def disabledPeriodic(self) -> None:
        self.subsystems.disable()
