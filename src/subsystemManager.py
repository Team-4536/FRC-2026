from dataclasses import dataclass, fields
from subsystems import networkTablesMixin as nt
from subsystems.cameras import CameraManager
from subsystems.inputs import Inputs
from subsystems.intake import Intake
from subsystems.LEDSignals import LEDSignals
from subsystems.networkTablesMixin import NetworkTablesMixin
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

    def phaseInit(self, state: RobotState) -> None:
        for s in self:
            s.phaseInit(state)

    def periodic(self, state: RobotState) -> None:
        for s in self:
            s.periodic(state)

    def disabled(self) -> None:
        for s in self:
            s.disabled()

    def publish(self) -> None:
        for s in self:
            s.publish()


@dataclass
class SubsystemManager(NetworkTablesMixin):
    subsystems: Subsystems
    inputs: Inputs
    cameras: CameraManager
    time: TimeData
    robotState: RobotState

    RUN_PUBLISH: bool = True
    DEBUGGING: bool = False

    def __post_init__(self) -> None:
        super().__init__(table="SubsystemManager", inst=False)

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

        self.publishBoolean("runPublish", self.RUN_PUBLISH)
        for s in self.subsystems:
            self.publishBoolean(
                f"publish{s.__class__.__name__}?", True, "specific", "subsystems"
            )
        for i in self:
            if not isinstance(i, Subsystems):
                self.publishBoolean(f"publish{i.__class__.__name__}?", True, "specific")

        for s in self:
            s.disabled()  # TODO: have subsystems make sure that their class attributes are initialized on class initialization
            self._publish(True)
        nt.debugging = self.DEBUGGING
        self.publishBoolean("debugging", self.DEBUGGING)

    def __iter__(self) -> Generator[Union[Subsystem, Subsystems]]:
        for f in fields(self):
            v = getattr(self, f.name)
            if isinstance(v, (Subsystem, Subsystems)):
                yield v

    def init(self) -> None:
        for s in self:
            s.phaseInit(self.robotState)

    def robotPeriodic(self) -> None:
        self._publish()
        self.robotState.publish()

        self.cameras.periodic(self.robotState)
        self.time.periodic(self.robotState)

    def autonomousPeriodic(self) -> None:
        self.inputs.periodic(self.robotState)
        self._periodic()

    def teleopPeriodic(self) -> None:
        self.inputs.periodic(self.robotState)
        self._periodic()

    def disabled(self) -> None:
        for s in self:
            s.disabled()

    def _periodic(self) -> None:
        for s in self.subsystems:
            s.periodic(self.robotState)

    def _publish(self, force: bool = False) -> None:
        if not force:
            nt.debugging = self.getBoolean("debugging", inst=False, default=False)
            if not self.getBoolean("runPublish", inst=False, default=False):
                return
        for i in self:
            if not isinstance(i, Subsystems):
                if self.getBoolean(
                    f"publish{i.__class__.__name__}?",
                    None,
                    "specific",
                    inst=False,
                    default=True,
                ):
                    i.publish()
            for s in self.subsystems:
                if self.getBoolean(
                    f"publish{s.__class__.__name__}?",
                    None,
                    "specific",
                    "subsystems",
                    inst=False,
                    default=True,
                ):
                    s.publish()
