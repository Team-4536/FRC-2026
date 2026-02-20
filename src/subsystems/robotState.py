from dataclasses import dataclass, fields, MISSING
from subsystems.networkTablesMixin import NetworkTablesMixin
from typing import Any, Self
from wpilib import Field2d, SmartDashboard
from wpimath.estimator import SwerveDrive4PoseEstimator
from wpimath.geometry import Pose2d
from wpimath.kinematics import ChassisSpeeds


@dataclass
class RobotState(NetworkTablesMixin):
    fieldSpeeds: ChassisSpeeds
    initialIntake: bool
    intakeSensorTest: bool
    intakeEject: bool
    intakePosYAxis = 0.0
    intakePos: bool
    intakeMode: bool
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
