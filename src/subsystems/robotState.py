from dataclasses import dataclass, fields
from math import pi as PI, tau as TAU
import numpy as np
import math
from enum import Enum
from wpimath.estimator import SwerveDrive4PoseEstimator
from wpilib import Field2d, SmartDashboard
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
    abtainableMaxSpeed: MPS
    resetGyro: bool
    pose: Pose2d
    odometry: SwerveDrive4PoseEstimator
    motorDesiredState: float

    revSpeed: float
    kickShooter: bool
    optimalTurretAngle: radians
    targetDistance: meters
    targetHeight: meters

    turretSwitchManual: bool
    turretManualSetpoint: float
    fullyreved: bool
    targetLocked: bool
    turretSwitchTarget: bool
    turretSwitchEnabled: bool
    turretResetYawEncdoer: bool

    robotOmegaSpeed: MPS
    robotLinearVelocity: Translation2d

    teamSide: TeamSide = TeamSide.SIDE_RED
    turretTarget: TurretTarget = TurretTarget.NONE
    turretMode: TurretMode = TurretMode.MANUAL


    def __post_init__(self) -> None:
        self.myField: Field2d = Field2d()
        SmartDashboard.putData("Field", self.myField)
        super().__init__()

    def publish(self) -> None:
        self.publishBoolean("Turret Manual", self.turretSwitchManual)
        for field in fields(self):
            name = field.name
            value = getattr(self, name)

            if isinstance(value, int):
                self.publishInteger(name, value)
            elif isinstance(value, str):
                self.publishString(name, value)
            elif isinstance(value, float):
                self.publishFloat(name, value)
            elif isinstance(value, bool):
                self.publishBoolean(name, value)

        self.publishFloat("vx", self.fieldSpeeds.vx, "FieldSpeeds")
        self.publishFloat("vy", self.fieldSpeeds.vy, "FieldSpeeds")
        self.publishFloat("omega", self.fieldSpeeds.omega, "FieldSpeeds")
        self.myField.setRobotPose(self.odometry.getEstimatedPosition())

        self.publishFloat(
            "x", metersToFeet(self.odometry.getEstimatedPosition().X()), "odom"
        )
        self.publishFloat(
            "y", metersToFeet(self.odometry.getEstimatedPosition().Y()), "odom"
        )
        self.publishFloat(
            "angle", self.odometry.getEstimatedPosition().rotation().degrees(), "odom"
        )

    @classmethod
    def empty(cls, **kwargs: Any) -> Self:
        data = {f.name: None for f in fields(cls)}

        data.update(kwargs)

        return cls(**data)  # type: ignore


def getTangentalVelocity(posFromCenter: Translation2d, speed: MPS) -> Translation2d:
    # TODO idk if this works
    tangentAngle: radians = math.atan(posFromCenter.y / posFromCenter.x) + PI / 2

    tangentVelocity: Translation2d = Translation2d(
        speed * math.cos(tangentAngle), speed * math.sin(tangentAngle)
    )

    return tangentVelocity


def getContributedRotation(tangentVel: Translation2d, angle: radians) -> MPS:
    tangentAngle = tangentVel.angle().radians()
    speed = tangentVel.distance(Translation2d())
    contributedVector: radians = math.cos(tangentAngle - angle)

    return speed * contributedVector


def RPMToMPS(speed: RPM, circ: meters) -> MPS:
    return speed / 60 * circ


def MPSToRPM(speed: MPS, circ: meters) -> RPM:
    return speed / circ * 60


def scaleTranslation2D(translation: Translation2d, scalar) -> Translation2d:

    angle = translation.angle().radians()
    hyp = translation.distance(Translation2d())
    xScale = hyp * math.cos(angle)
    yScale = hyp * math.sin(angle)

    return Translation2d(xScale, yScale)


def wrapAngle(angle: radians) -> radians:

    # mod only returns positive like a bum
    if angle == 0:
        return 0

    wrappedAngle: radians = angle % (TAU * (angle / abs(angle)))
    return wrappedAngle


def RPMToVolts(rpm: RPM, maxRPM) -> float:

    return rpm / (maxRPM / BATTERY_VOLTS)
