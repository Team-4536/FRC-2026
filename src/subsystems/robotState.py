from dataclasses import dataclass, fields, MISSING
from enum import Enum
from subsystems.networkTablesMixin import NetworkTablesMixin
from typing import Any, Self
from wpilib import Field2d, SmartDashboard
from wpimath.estimator import SwerveDrive4PoseEstimator
from wpimath.geometry import Translation2d
from wpimath.kinematics import ChassisSpeeds
from wpimath.units import inchesToMeters, meters_per_second, meters, radians

ROBOT_RADIUS = inchesToMeters(11)  # TODO idk the actual thing
BATTERY_VOLTS: float = 12


class TurretTarget(Enum):
    HUB = 1
    SHUTTLE_TOP = 2
    SHUTTLE_BOTTOM = 3


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
    indexerEject: bool
    intakePos: bool
    intakeMode: bool
    resetGyro: bool
    odometry: SwerveDrive4PoseEstimator

    revSpeed: float
    kickShooter: int
    turretSwitchMode: bool
    turretManualSetpoint: float
    turretSwitchTarget: bool
    turretSwitchEnabled: bool
    dontShoot: bool
    impossibleDynamic: bool
    forceDynamicTurret: bool
    turretVelocitySetpoint: Translation2d

    robotOmegaSpeed: meters_per_second
    robotLinearVelocity: Translation2d

    turretTarget: TurretTarget = TurretTarget.HUB
    turretMode: TurretMode = TurretMode.MANUAL
    ejectAll: float = 0.0
    intakePosYAxis: float = 0.0

    def __post_init__(self) -> None:
        super().__init__(table="RobotState", inst=False)
        self.odomField: Field2d = Field2d()
        SmartDashboard.putData("odomField", self.odomField)

    def publish(self) -> None:
        for field in fields(self):
            name = field.name
            value = getattr(self, name)
            self.publishGeneric(name, value)

        self.odomField.setRobotPose(self.odometry.getEstimatedPosition())

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
