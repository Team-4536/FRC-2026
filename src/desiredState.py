from dataclasses import dataclass
from wpimath.kinematics import ChassisSpeeds


@dataclass
class DesiredState:
    fieldSpeeds: ChassisSpeeds
