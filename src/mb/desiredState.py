from dataclasses import dataclass
from wpimath.kinematics import ChassisSpeeds
from mb.networkTablesMixin import NetworkTablesMixin


@dataclass
class DesiredState(NetworkTablesMixin):
    fieldSpeeds: ChassisSpeeds

    def init(self):
        super().__init__(instance="RobotDesiredState")

    def publish(self):
        vx = float(self.fieldSpeeds.vx)
        vy = float(self.fieldSpeeds.vy)
        omega = float(self.fieldSpeeds.omega)

        self.publishDouble("vx", vx)
        self.publishDouble("vy", vy)
        self.publishDouble("omega", omega)
