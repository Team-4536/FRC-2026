from desiredState import DesiredState
from drive import SwerveDrive
from inputs import Inputs
from LEDSignals import LEDSignals
from motor import RevMotor
from ntcore import NetworkTable, NetworkTableInstance
from subsystem import Subsystem
from typing import List, NamedTuple
from utils import TimeData
from wpilib import run, TimedRobot


class Subsystems(NamedTuple):
    inputs: Inputs
    ledSignals: LEDSignals
    swerveDrive: SwerveDrive
    time: TimeData

    def init(self):
        for s in self:
            s.init()

    def periodic(self):
        ds = self.desiredState
        for s in self._s:
            s.periodic(ds)

    def disable(self):
        for s in self:
            s.disable()

    @property
    def _s(self) -> List[Subsystem]:
        return [
            self.ledSignals,
            self.swerveDrive,
            self.time,
        ]

    @property
    def desiredState(self) -> DesiredState:
        self.inputs.periodic(self.inputs.desiredState)
        return self.inputs.desiredState


class Robot(TimedRobot):
    def robotInit(self) -> None:
        self.table: NetworkTable = NetworkTableInstance.getDefault().getTable("telemetry")

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


if __name__ == "__main__":
    run(Robot)  # type: ignore
