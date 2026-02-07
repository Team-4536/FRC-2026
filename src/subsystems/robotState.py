from dataclasses import dataclass, fields
from math import pi as PI
import math
from subsystems.networkTablesMixin import NetworkTablesMixin
from wpimath.geometry import Pose2d, Translation2d
from wpimath.kinematics import ChassisSpeeds
from wpimath.units import meters_per_second as MPS
from wpimath.units import revolutions_per_minute as RPM
from wpimath.units import metersToFeet, radians, meters, inchesToMeters

ROBOT_RADIUS = inchesToMeters(11)  # TODO idk the actual thing
from typing import Any, Self


@dataclass
class RobotState(NetworkTablesMixin):
    fieldSpeeds: ChassisSpeeds
    abtainableMaxSpeed: MPS
    resetGyro: bool
    pose: Pose2d
    motorDesiredState: float

    revShooter: RPM
    revSpeed: RPM
    kickShooter: RPM
    optimalTurretAngle: radians
    hubDistance: meters

    turretManualToggle: bool
    turretManulMode: bool
    turretManualSetpoint: float

    robotOmegaVelocity: MPS
    robotLinearVelocity: Translation2d

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


def getTangentalVelocity(
    posX: meters, posY: meters, angle: radians, speed: MPS
) -> MPS:  # TODO change to a translation2D for angle relative to feild
    tangentAngle: radians = math.atan(posY / posX) + PI / 2
    actualAngle: radians = math.cos(tangentAngle - angle)

    tangentvelocity: MPS = speed * actualAngle

    return tangentvelocity
