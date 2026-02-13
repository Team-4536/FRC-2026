from dataclasses import dataclass, fields
from math import pi as PI, tau as TAU
import numpy as np
import math
from enum import Enum

from subsystems.networkTablesMixin import NetworkTablesMixin
from wpimath.geometry import Pose2d, Translation2d
from wpimath.kinematics import ChassisSpeeds
from wpimath.units import meters_per_second as MPS
from wpimath.units import revolutions_per_minute as RPM
from wpimath.units import (
    metersToFeet,
    radians,
    meters,
    inchesToMeters,
    rotationsPerMinuteToRadiansPerSecond,
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
    motorDesiredState: float

    revSpeed: RPM
    kickShooter: RPM
    optimalTurretAngle: radians
    targetDistance: meters
    targetHeight: meters

    turretManualToggle: bool
    turretManualSetpoint: float
    fullyreved: bool
    targetLocked: bool
    turretSwitchTarget: bool
    putAwayTurret: bool

    robotOmegaSpeed: MPS
    robotLinearVelocity: Translation2d

    teamSide: TeamSide = TeamSide.SIDE_RED
    turretTarget: TurretTarget = TurretTarget.NONE
    turretMode: TurretMode = TurretMode.MANUAL

    def __post_init__(self) -> None:
        super().__init__()

    def publish(self) -> None:
        self.publishBoolean("Turret Manual", self.turretManualToggle)
        for field in fields(self):
            name = field.name
            value = getattr(self, name)

            self.publishGeneric(name, value)

        self.publishFloat("vx", self.fieldSpeeds.vx, "FieldSpeeds")
        self.publishFloat("vy", self.fieldSpeeds.vy, "FieldSpeeds")
        self.publishFloat("omega", self.fieldSpeeds.omega, "FieldSpeeds")

        if self.pose:
            self.publishFloat("x", metersToFeet(self.pose.X()), "odom")
            self.publishFloat("y", metersToFeet(self.pose.Y()), "odom")
            self.publishFloat("angle", self.pose.rotation().degrees(), "odom")

    @classmethod
    def empty(cls, **kwargs: Any) -> Self:
        data = {f.name: None for f in fields(cls)}

        data.update(kwargs)  # test

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


def RPMToMPS(speed: RPM, circ: meters):
    return rotationsPerMinuteToRadiansPerSecond(speed) * circ


def scaleTranslation2D(translation: Translation2d, scalar) -> Translation2d:

    angle = translation.angle().radians()
    hyp = translation.distance(Translation2d())
    xScale = hyp * math.cos(angle)
    yScale = hyp * math.sin(angle)

    return Translation2d(xScale, yScale)


def wrapAngle(angle: radians) -> radians:

    # mod only returns positive like a bum
    wrappedAngle: radians = (angle % TAU) * np.sign(angle)
    return wrappedAngle


def RPMToVolts(rpm: RPM, maxRPM) -> float:

    return rpm / (maxRPM / BATTERY_VOLTS)
