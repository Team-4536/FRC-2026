from dataclasses import dataclass, fields
from subsystems.networkTablesMixin import NetworkTablesMixin
from wpimath.geometry import Pose2d
from wpimath.kinematics import ChassisSpeeds
from wpimath.units import meters_per_second as MPS


@dataclass
class RobotState(NetworkTablesMixin):
    fieldSpeeds: ChassisSpeeds
    abtainableMaxSpeed: MPS
    pose: Pose2d

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

        vx = float(self.fieldSpeeds.vx)
        vy = float(self.fieldSpeeds.vy)
        omega = float(self.fieldSpeeds.omega)

        self.publishDouble("vx", vx, "FieldSpeeds")
        self.publishDouble("vy", vy, "FieldSpeeds")
        self.publishDouble("omega", omega, "FieldSpeeds")

    @classmethod
    def empty(cls, **kwargs):
        data = {f.name: None for f in fields(cls)}

        data.update(kwargs)

        return cls(**data)  # type: ignore
