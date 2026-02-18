from dataclasses import dataclass, fields
from subsystems.networkTablesMixin import NetworkTablesMixin
from typing import Any, Self
from wpilib import Field2d, SmartDashboard
from wpimath.estimator import SwerveDrive4PoseEstimator
from wpimath.geometry import Pose2d
from wpimath.kinematics import ChassisSpeeds


@dataclass
class RobotState(NetworkTablesMixin):
    fieldSpeeds: ChassisSpeeds
    resetGyro: bool
    pose: Pose2d
    odometry: SwerveDrive4PoseEstimator

    def __post_init__(self) -> None:
        self.myField: Field2d = Field2d()
        SmartDashboard.putData("Field", self.myField)
        super().__init__()

    def publish(self) -> None:
        for field in fields(self):
            name = field.name
            value = getattr(self, name)
            self.publishGeneric(name, value)

        self.myField.setRobotPose(self.odometry.getEstimatedPosition())

    @classmethod
    def empty(cls, **kwargs: Any) -> Self:
        data = {f.name: None for f in fields(cls)}

        data.update(kwargs)

        return cls(**data)  # type: ignore
