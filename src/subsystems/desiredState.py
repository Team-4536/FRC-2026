from dataclasses import dataclass
from subsystems.networkTablesMixin import NetworkTablesMixin
from wpimath.kinematics import ChassisSpeeds
from wpimath.units import meters_per_second as MPS


@dataclass
class DesiredState(NetworkTablesMixin):
    fieldSpeeds: ChassisSpeeds
    abtainableMaxSpeed: MPS
    turretSpeed: float

    def __post_init__(self) -> None:
        super().__init__()

    def publish(self) -> None:
        vx = float(self.fieldSpeeds.vx)
        vy = float(self.fieldSpeeds.vy)
        omega = float(self.fieldSpeeds.omega)

        self.publishDouble("vx", vx, "FieldSpeeds")
        self.publishDouble("vy", vy, "FieldSpeeds")
        self.publishDouble("omega", omega, "FieldSpeeds")
