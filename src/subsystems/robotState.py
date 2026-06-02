from dataclasses import dataclass, field, fields, MISSING
from enum import Enum
from subsystems.networkTablesMixin import NetworkTablesMixin
from typing import Any, Self
from wpilib import Field2d, SmartDashboard
from wpimath.estimator import SwerveDrive4PoseEstimator
from wpimath.geometry import Pose2d, Translation2d, Rotation2d
from wpimath.kinematics import ChassisSpeeds


class TurretTarget(Enum):
    HUB = 0
    SHUTTLE_TOP = 1
    SHUTTLE_BOTTOM = 2


class TurretMode(Enum):
    DISABLED = 0
    MANUAL = 1
    DYNAMIC = 2


class ClimberState(Enum):
    DISABLED = 0
    CLIMB_UP = 1
    CLIMB_DOWN = 2


def default(default: Any) -> Any:
    return field(default_factory=lambda: default)


@dataclass
class RobotState(NetworkTablesMixin):
    limelightPose: Pose2d | None
    odometry: SwerveDrive4PoseEstimator

    # Drive
    gyro: Rotation2d = default(Rotation2d())
    fieldSpeeds: ChassisSpeeds = default(ChassisSpeeds())
    robotVelocity: ChassisSpeeds = default(ChassisSpeeds())
    resetGyro: bool = False

    # Climb
    climbState: ClimberState = ClimberState.DISABLED

    # Turret
    assistedTurret: bool = False  # set to a button evetually
    dontShoot: bool = False
    kickShooter: int = 0
    revSpeed: float = 0.0
    kickerEject: bool = False
    turretVelocitySetpoint: Translation2d = default(Translation2d())
    turretManualSetpoint: float = 0.0
    turretMode: TurretMode = TurretMode.MANUAL
    turretSwitchMode: bool = False
    turretSwitchTarget: bool = False

    # Intake
    indexerEject: bool = False
    initialIntake: bool = False
    intakeEject: bool = False
    intakeIndexer: bool = False
    intakeModeLeftBumperPressed: bool = False
    intakePos: bool = False
    intakePosYAxis: float = 0.0

    # Autonomous
    autosGyroReset: float = 0.0
    autosGyroResetToggle: bool = False
    autosInitPose: Pose2d = default(Pose2d())
    slowdown: float = 1
    mode: str = "comp"

    # Other
    ejectAll: bool = False

    def __post_init__(self) -> None:
        super().__init__(table="RobotState")
        self.publishFloat("slowdown", self.slowdown)
        self.publishString("Mode", self.mode)
        self.odomField: Field2d = Field2d()
        SmartDashboard.putData("Field", self.odomField)

    def publish(self) -> None:
        for field in fields(self):
            name = field.name
            value = getattr(self, name)
            if value is not None:
                self.publishAny(name, value)

        robotPose = self.odometry.getEstimatedPosition()
        self.odomField.setRobotPose(robotPose)
        if self.limelightPose != None:
            self.odomField.setRobotPose(self.limelightPose)
        self.publishStruct("robotPosition", robotPose)

    @classmethod
    def empty(cls, **kwargs: Any) -> Self:
        data = {}
        for f in fields(cls):
            if f.name in kwargs:
                data[f.name] = kwargs[f.name]
            elif f.default is not MISSING:
                data[f.name] = f.default
            elif f.default_factory is not MISSING:
                data[f.name] = f.default_factory()
            else:
                data[f.name] = None

        return cls(**data)  # pyright: ignore
