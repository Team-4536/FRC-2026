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

        x, y, angle = 0, 0, 0
        if self.pose:
            x = metersToFeet(self.pose.X())
            y = metersToFeet(self.pose.Y())
            angle = self.pose.rotation().degrees()
        self.publishFloat("x", x, "odom")
        self.publishFloat("y", y, "odom")
        self.publishFloat("angle", angle, "odom")

    @classmethod
    def empty(cls, **kwargs: Any) -> Self:
        data = {f.name: None for f in fields(cls)}

        data.update(kwargs)

        return cls(**data)  # type: ignore
