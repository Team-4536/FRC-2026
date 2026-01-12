from subsystems.desiredState import DesiredState
from subsystems.inputs import Inputs
from subsystems.LEDSignals import LEDSignals
from subsystems.motor import RevMotor
from subsystems.subsystem import Subsystem
from subsystems.swerveDrive import SwerveDrive
from subsystems.utils import TimeData
from typing import List, NamedTuple
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


class Subsystems(NamedTuple):
    inputs: Inputs
    ledSignals: LEDSignals
    swerveDrive: SwerveDrive
    time: TimeData

    def init(self) -> None:
        for s in self:
            s.init()

    def periodic(self) -> None:
        ds = self.desiredState
        for s in self._dependant:
            s.periodic(ds)
        self.publish()

    def disable(self) -> None:
        for s in self:
            s.disabled()

    def publish(self) -> None:
        for s in self:
            s.publish()

    @property
    def _dependant(self) -> List[Subsystem]:
        return [
            self.ledSignals,
            self.swerveDrive,
            self.time,
        ]

    @property
    def desiredState(self) -> DesiredState:
        self.inputs.periodic(self.inputs.desiredState)
        return self.inputs.desiredState
