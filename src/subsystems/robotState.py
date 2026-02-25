from dataclasses import dataclass, fields, MISSING
from math import pi as PI, tau as TAU
import numpy as np
import math
from enum import Enum
from wpimath.estimator import SwerveDrive4PoseEstimator
from wpilib import Field2d, SmartDashboard, SendableChooser
from ntcore import NetworkTable
from subsystems.networkTablesMixin import NetworkTablesMixin
from wpimath.geometry import Pose2d, Translation2d, Pose3d
from wpimath.kinematics import ChassisSpeeds
from wpimath.units import meters_per_second as MPS
from wpimath.units import revolutions_per_minute as RPM
from wpimath.units import (
    metersToFeet,
    radians,
    meters,
    inchesToMeters,
)
from typing import Any, Self

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
    pose: Pose2d
    odometry: SwerveDrive4PoseEstimator
    motorDesiredState: float

    revSpeed: float
    kickShooter: bool
    optimalTurretAngle: radians
    targetDistance: meters
    targetHeight: meters

    turretSwitchMode: bool
    turretManualSetpoint: float
    fullyreved: bool
    targetLocked: bool
    turretSwitchTarget: bool
    turretSwitchEnabled: bool
    turretResetYawEncdoer: bool
    dontShoot: bool
    impossibleDynamic: bool
    forceDynamicTurret: bool

    robotOmegaSpeed: MPS
    robotLinearVelocity: Translation2d

    teamSide: TeamSide = TeamSide.SIDE_BLUE
    turretTarget: TurretTarget = TurretTarget.NONE
    turretMode: TurretMode = TurretMode.MANUAL
    ejectAll = 0.0
    intakePosYAxis = 0.0

    def __post_init__(self) -> None:
        self.myField: Field2d = Field2d()
        SmartDashboard.putData("Field2", self.myField)
        # SendableChooser().addOption("SIDE_RED")
        super().__init__()
        self.publish

    def publish(self) -> None:
        for field in fields(self):
            name = field.name
            value = getattr(self, name)
            self.publishGeneric(name, value)

            self.publishGeneric(name, value)

        self.publishFloat("vx", self.fieldSpeeds.vx, "FieldSpeeds")
        self.publishFloat("vy", self.fieldSpeeds.vy, "FieldSpeeds")
        self.publishFloat("omega", self.fieldSpeeds.omega, "FieldSpeeds")
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

        return cls(**data)  # type: ignore
