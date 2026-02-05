from dataclasses import dataclass, fields
from subsystems.networkTablesMixin import NetworkTablesMixin
from typing import Any, Self
from wpimath.geometry import Pose2d
from wpimath.kinematics import ChassisSpeeds
from wpimath.units import meters_per_second as MPS
from wpimath.units import metersToFeet


@dataclass
class RobotState(NetworkTablesMixin):
    fieldSpeeds: ChassisSpeeds
    abtainableMaxSpeed: MPS
    resetGyro: bool
    pose: Pose2d
    extended: bool
    contracted: bool
    buttonUp: bool
    buttonDown: bool

    def __post_init__(self) -> None:
        super().__init__()

    def publish(self) -> None:
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
