from dataclasses import dataclass, fields
from ntcore import NetworkTableInstance
from subsystems import networkTablesMixin as nt
from subsystems.autoSubsystem import AutoSubsystem
from subsystems.cameras import CameraManager
from subsystems.climber import Climber
from subsystems.inputs import Inputs
from subsystems.intake import Intake
from subsystems.limelights import llCams
from subsystems.networkTablesMixin import NetworkTablesMixin
from subsystems.robotState import RobotState
from subsystems.subsystem import Subsystem
from subsystems.swerveDrive import SwerveDrive
from subsystems.tester import Tester
from subsystems.turretSystem import Turret, Shooter
from subsystems.utils import matchData, TimeData
from typing import Generator, NamedTuple, Union
from wpimath.estimator import SwerveDrive4PoseEstimator
from wpimath.geometry import Pose2d, Rotation2d
from wpimath.kinematics import ChassisSpeeds

table = NetworkTableInstance.getDefault().getTable("profiling")
table.putNumber("OVERRUN!!!", 20)


class Subsystems(NamedTuple):
    intake: Intake
    shooter: Shooter
    swerveDrive: SwerveDrive
    turret: Turret
    climb: Climber

    def phaseInit(self, state: RobotState) -> None:
        for s in self:
            s.phaseInit(state)

    def periodic(self, state: RobotState) -> None:
        totalTime = 0
        for s in self:
            startTime = matchData.timeSinceInit
            s.periodic(state)
            time = (matchData.timeSinceInit - startTime) * 1000
            totalTime += time
            table.putNumber(s.__class__.__name__, time)
        table.putNumber("total_time", totalTime)

    def robotPeriodic(self, state: RobotState) -> None:
        for s in self:
            s.robotPeriodic(state)

    def disabled(self) -> None:
        totalTime = 0
        for s in self:
            startTime = matchData.timeSinceInit
            s.disabled()
            time = (matchData.timeSinceInit - startTime) * 1000
            table.putNumber(s.__class__.__name__, time)
            totalTime += time
        table.putNumber("total_time", totalTime)


@dataclass
class SubsystemManager(NetworkTablesMixin):
    inputs: Inputs
    autos: AutoSubsystem
    tests: Tester
    cameras: CameraManager
    llCam: llCams
    time: TimeData
    subsystems: Subsystems
    robotState: RobotState

    RUN_PUBLISH: bool = True
    DEBUGGING: bool = False

    def __post_init__(self) -> None:
        super().__init__(table="SubsystemManager")

        drive = self.subsystems.swerveDrive
        initPos = (
            Pose2d(x=2, y=4, rotation=Rotation2d())
            if matchData.isBlue()
            else Pose2d(x=14.5, y=4, rotation=Rotation2d.fromDegrees(180))
        )

        self.robotState.fieldSpeeds = ChassisSpeeds()
        self.robotState.odometry = SwerveDrive4PoseEstimator(
            drive.kinematics,
            self.robotState.gyro,
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

        self.disabled()  # TODO: have subsystems make sure that their class attributes are initialized on class initialization
        for s in self:
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
        self.subsystems.robotPeriodic(self.robotState)
        self.robotState.slowdown = self.robotState.getFloat("slowdown", default=1)
        self.robotState.mode = self.robotState.getString("Mode", default="comp")
        self.cameras.periodic(self.robotState)

        self.llCam.periodic(self.robotState)
        self.time.periodic(self.robotState)

        self._publish()
        self.robotState.publish()

    def autonomousPeriodic(self) -> None:
        self.autos.periodic(self.robotState)
        self.subsystems.periodic(self.robotState)

    def teleopPeriodic(self) -> None:
        self.inputs.periodic(self.robotState)
        self.subsystems.periodic(self.robotState)

    def testPeriodic(self) -> None:
        self.tests.periodic(self.robotState)
        self.subsystems.periodic(self.robotState)

    def disabled(self) -> None:
        for s in self:
            s.disabled()

    # TODO: maybe move some of this back into Subsystems
    def _publish(self, force: bool = False) -> None:
        if not force:
            nt.debugging = self.getBoolean("debugging", default=False)
            if not self.getBoolean("runPublish", default=False):
                return
        for i in self:
            if not isinstance(i, Subsystems):
                # if self.getBoolean(
                #     f"publish{i.__class__.__name__}?", None, "specific", default=True
                # ):
                i.publish()
                continue
            for s in self.subsystems:
                # if self.getBoolean(
                #     f"publish{s.__class__.__name__}?",
                #     None,
                #     "specific",
                #     "subsystems",
                #     default=True,
                # ):
                s.publish()
