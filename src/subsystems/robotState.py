from dataclasses import dataclass, fields
from subsystems.networkTablesMixin import NetworkTablesMixin
from wpimath.geometry import Pose2d
from wpimath.kinematics import ChassisSpeeds
from wpimath.units import meters_per_second as MPS
from wpimath.units import revolutions_per_minute as RPM
from wpimath.units import metersToFeet, radians, meters


@dataclass
class RobotState(NetworkTablesMixin):
    fieldSpeeds: ChassisSpeeds
    abtainableMaxSpeed: MPS
    pose: Pose2d
    motorDesiredState: float

    revShooter: RPM
    shootShooter: RPM
    optimalTurretAngle: radians
    hubDistance: meters

    limitA: bool
    limitB: bool

    def __post_init__(self) -> None:
        super().__init__()

    def publish(self) -> None:
        for field in fields(self):
            name = field.name
            value = getattr(self, name)

            if isinstance(value, int):
                self.publishInteger(name, value)
            elif isinstance(value, str):
                self.publishString(name, value)
            elif isinstance(value, float):
                self.publishDouble(name, value)
            elif isinstance(value, bool):
                self.publishBoolean(name, value)

        self.publishDouble("vx", self.fieldSpeeds.vx, "FieldSpeeds")
        self.publishDouble("vy", self.fieldSpeeds.vy, "FieldSpeeds")
        self.publishDouble("omega", self.fieldSpeeds.omega, "FieldSpeeds")

        if self.pose:
            self.publishDouble("x", metersToFeet(self.pose.X()), "odom")
            self.publishDouble("y", metersToFeet(self.pose.Y()), "odom")
            self.publishDouble("angle", self.pose.rotation().degrees(), "odom")

    @classmethod
    def empty(cls, **kwargs):
        data = {f.name: None for f in fields(cls)}

        data.update(kwargs)

        return cls(**data)  # type: ignore
