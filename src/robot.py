from mb.inputs import Inputs
from mb.LEDSignals import LEDSignals
from mb.motor import RevMotor
from mb.swerveDrive import SwerveDrive
from mb.utils import TimeData
from subsystems import Subsystems
from wpilib import TimedRobot


class Robot(TimedRobot):
    def robotInit(self) -> None:
        self.subsystems: Subsystems = Subsystems(
            inputs=Inputs(),
            ledSignals=LEDSignals(deviceID=0),
            swerveDrive=SwerveDrive.symmetricDrive(
                flDriveID=2,
                flAzimuthID=1,
                frDriveID=4,
                frAzimuthID=3,
                blDriveID=6,
                blAzimuthID=5,
                brDriveID=8,
                brAzimuthID=7,
                driveGearing=6.12,
                azimuthGearing=21.4,
                xPos=1,
                yPos=1,
            ),
            time=TimeData(),
        )

        self.subsystems.swerveDrive.configureDriveMotors(config=RevMotor.driveConfig)
        self.subsystems.swerveDrive.configureAzimuthMotors(config=RevMotor.azimuthConfig)

    def robotPeriodic(self) -> None:
        self.subsystems.time.periodic(self.subsystems.desiredState)

    def teleopInit(self) -> None:
        for s in self.subsystems:
            s.init()

    def teleopPeriodic(self) -> None:
        self.subsystems.periodic()

    def disabledInit(self) -> None:
        self.disabledPeriodic()

    def disabledPeriodic(self) -> None:
        self.subsystems.disable()
