from dataclasses import dataclass, fields, MISSING
from enum import Enum
from subsystems.networkTablesMixin import NetworkTablesMixin
from typing import Any, Self
from wpilib import Field2d, SmartDashboard
from wpimath.estimator import SwerveDrive4PoseEstimator
from wpimath.geometry import Translation2d
from wpimath.kinematics import ChassisSpeeds
from wpimath.units import (
    radians,
    meters_per_second,
    meters,
    inchesToMeters,
)

ROBOT_RADIUS = inchesToMeters(11)  # TODO idk the actual thing
BATTERY_VOLTS: float = 12


class TeamSide(Enum):
    SIDE_RED = 1
    SIDE_BLUE = 2


class TurretTarget(Enum):
    NONE = 0
    HUB = 1
    SHUTTLE_RIGHT = 2
    SHUTTLE_LEFT = 3


class TurretMode(Enum):
    DISABLED = 0
    MANUAL = 1
    DYNAMIC = 2


@dataclass
class RobotState(NetworkTablesMixin):
    fieldSpeeds: ChassisSpeeds
    initialIntake: bool
    intakeIndexer: bool
    intakeEject: bool
    intakePos: int
    intakeMode: bool
    resetGyro: bool
    odometry: SwerveDrive4PoseEstimator

    revSpeed: float
    kickShooter: bool
    optimalTurretAngle: radians  # REMOVE (local var)
    targetDistance: meters  # REMOVE (local var)
    targetHeight: meters  # REMOVE (local var)
    turretSwitchMode: bool
    turretManualSetpoint: float
    turretSwitchTarget: bool
    turretSwitchEnabled: bool
    turretResetYawEncdoer: bool  # REMOVE (local var)
    dontShoot: bool  # REMOVE (local var)
    impossibleDynamic: bool  # REMOVE (local var)
    forceDynamicTurret: bool  # REMOVE (local var)
    dontShoot: bool  # REMOVE (local var)

    robotOmegaSpeed: meters_per_second
    robotLinearVelocity: Translation2d

    teamSide: TeamSide = TeamSide.SIDE_RED  # MAYBE REMOVE LATER
    turretMode: TurretMode = TurretMode.MANUAL  # REMOVE (local var)
    ejectAll = 0.0
    intakePosYAxis = 0.0

    def __post_init__(self) -> None:
        self.myField: Field2d = Field2d()
        SmartDashboard.putData("odomField", self.myField)
        super().__init__()

    def publish(self) -> None:
        for field in fields(self):
            name = field.name
            value = getattr(self, name)
            self.publishGeneric(name, value)

        self.myField.setRobotPose(self.odometry.getEstimatedPosition())

        self.publishFloat(
            "Robot Angle DJO", self.odometry.getEstimatedPosition().rotation().radians()
        )

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
