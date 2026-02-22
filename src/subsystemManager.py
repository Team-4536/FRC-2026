from dataclasses import dataclass, fields
from subsystems.cameras import CameraManager
from subsystems.inputs import Inputs
from subsystems.LEDSignals import LEDSignals
from subsystems.robotState import RobotState
from subsystems.subsystem import Subsystem
from subsystems.swerveDrive import SwerveDrive
from subsystems.utils import TimeData
from typing import Generator, NamedTuple
from wpimath.estimator import SwerveDrive4PoseEstimator
from wpimath.kinematics import ChassisSpeeds


class Subsystems(NamedTuple):
    ledSignals: LEDSignals
    swerveDrive: SwerveDrive

    def phaseInit(self, rs: RobotState) -> None:
        (s.phaseInit(rs) for s in self)

    def periodic(self, rs: RobotState) -> None:
        (s.periodic(rs) for s in self)

    def disabled(self) -> None:
        (s.disabled() for s in self)

    def publish(self) -> None:
        (s.publish() for s in self)


@dataclass(frozen=True)
class SubsystemManager:
    subsystems: Subsystems
    inputs: Inputs
    cameras: CameraManager
    time: TimeData
    robotState: RobotState

    def __post_init__(self) -> None:
        self.robotState.fieldSpeeds = ChassisSpeeds()
        swerve = self.subsystems.swerveDrive
        self.robotState.odometry = SwerveDrive4PoseEstimator(
            swerve.kinematics,
            swerve.roboAngle,
            swerve.modulePoses,
            swerve.initPos,
        )

    def __iter__(self) -> Generator[Subsystem]:
        for f in fields(self):
            v = getattr(self, f.name)
            if isinstance(v, Subsystem):
                yield v

    def init(self) -> None:
        for s in self.subsystems:
            s.phaseInit(self.robotState)
        self.inputs.phaseInit(self.robotState)
        self.cameras.phaseInit(self.robotState)
        self.time.phaseInit(self.robotState)

    def robotPeriodic(self) -> None:
        for s in self:
            s.publish()
        self.cameras.periodic(self.robotState)
        self.time.periodic(self.robotState)

    def autonomousPeriodic(self) -> None:
        self.inputs.periodic(self.robotState)
        self._periodic()

    def teleopPeriodic(self) -> None:
        self.inputs.periodic(self.robotState)
        self._periodic()

    def _periodic(self) -> None:
        for s in self.subsystems:
            s.periodic(self.robotState)

    def disabled(self) -> None:
        for s in (*self.subsystems, self.inputs, self.cameras, self.time):
            s.disabled()
