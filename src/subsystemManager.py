from dataclasses import dataclass, fields
from subsystems.cameras import CameraManager
from subsystems.inputs import Inputs
from subsystems.intake import Intake
from subsystems.LEDSignals import LEDSignals
from subsystems.robotState import RobotState
from subsystems.subsystem import Subsystem
from subsystems.swerveDrive import SwerveDrive
from subsystems.turretSystem import Turret, Shooter
from subsystems.utils import matchData, TimeData
from typing import Generator, NamedTuple, Union
from wpimath.estimator import SwerveDrive4PoseEstimator
from wpimath.geometry import Pose2d, Rotation2d
from wpimath.kinematics import ChassisSpeeds


class Subsystems(NamedTuple):
    intake: Intake
    ledSignals: LEDSignals
    shooter: Shooter
    swerveDrive: SwerveDrive
    turret: Turret

    def phaseInit(self, rs: RobotState) -> None:
        for s in self:
            s.phaseInit(rs)

    def periodic(self, rs: RobotState) -> None:
        for s in self:
            s.periodic(rs)

    def disabled(self) -> None:
        for s in self:
            s.disabled()

    def publish(self) -> None:
        for s in self:
            s.publish()


@dataclass
class SubsystemManager:
    subsystems: Subsystems
    inputs: Inputs
    cameras: CameraManager
    time: TimeData
    robotState: RobotState

    def __post_init__(self) -> None:
        drive = self.subsystems.swerveDrive
        initPos = (
            Pose2d(x=2, y=4, rotation=Rotation2d())
            if matchData.isBlue()
            else Pose2d(x=14.5, y=4, rotation=Rotation2d.fromDegrees(180))
        )

        self.robotState.fieldSpeeds = ChassisSpeeds()
        self.robotState.odometry = SwerveDrive4PoseEstimator(
            drive.kinematics,
            drive.roboAngle,
            drive.modulePoses,
            initPos,
        )

    def __iter__(self) -> Generator[Union[Subsystem, Subsystems]]:
        for f in fields(self):
            v = getattr(self, f.name)
            if isinstance(v, (Subsystem, Subsystems)):
                yield v

    def init(self) -> None:
        for s in self:
            s.phaseInit(self.robotState)

    def robotPeriodic(self) -> None:
        for s in self:
            s.publish()
        self.robotState.publish()

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
        for s in self:
            s.disabled()
