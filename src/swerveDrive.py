import math
from subsystem import Subsystem

from ntcore import NetworkTableInstance
from rev import SparkMax, SparkLowLevel
from typing import List, Tuple
from utils import wrapAngle
from wpimath.geometry import Pose2d, Rotation2d, Translation2d
from wpimath.kinematics import (
    ChassisSpeeds,
    SwerveDrive4Kinematics,
    SwerveDrive4Odometry,
    SwerveModulePosition,
    SwerveModuleState,
)


class SwerveDrive(Subsystem):
    def __init__(self) -> None:
        pass
