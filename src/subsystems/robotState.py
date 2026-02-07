from dataclasses import dataclass, fields
from subsystems.networkTablesMixin import NetworkTablesMixin
from wpimath.geometry import Pose2d, Pose3d
from wpimath.kinematics import ChassisSpeeds
from wpimath.units import meters_per_second as MPS
from wpimath.estimator import SwerveDrive4PoseEstimator
from wpimath.units import metersToFeet
from wpilib import Field2d
from wpilib import SmartDashboard


@dataclass
class RobotState(NetworkTablesMixin):
    fieldSpeeds: ChassisSpeeds
    abtainableMaxSpeed: MPS
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
        self.myField.setRobotPose(self.odometry.getEstimatedPosition())

        self.publishDouble(
            "x", metersToFeet(self.odometry.getEstimatedPosition().X()), "odom"
        )
        self.publishDouble(
            "y", metersToFeet(self.odometry.getEstimatedPosition().Y()), "odom"
        )
        self.publishDouble(
            "angle", self.odometry.getEstimatedPosition().rotation().degrees(), "odom"
        )

    @classmethod
    def empty(cls, **kwargs):
        data = {f.name: None for f in fields(cls)}

        data.update(kwargs)

        return cls(**data)  # type: ignore
