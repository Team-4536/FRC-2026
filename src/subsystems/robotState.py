from dataclasses import dataclass, fields, MISSING
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
    initialIntake: bool = False  # TODO: change vals later, very temp
    intakeSensorTest: bool = False
    intakeEject: bool = False
    intakePosYAxis: float = 1
    intakePos: bool = False
    intakeMode: bool = True

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
