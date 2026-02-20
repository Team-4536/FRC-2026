from dataclasses import dataclass, fields
from subsystems.networkTablesMixin import NetworkTablesMixin
from wpimath.geometry import Pose2d, Rotation2d
from wpimath.kinematics import ChassisSpeeds
from wpimath.units import meters_per_second as MPS
from wpilib import SmartDashboard
from wpilib import Field2d
from wpimath.units import metersToFeet
from ntcore import NetworkTable, NetworkTableInstance


@dataclass
class RobotState(NetworkTablesMixin):
    fieldSpeeds: ChassisSpeeds
    abtainableMaxSpeed: MPS
    pose: Pose2d
    limelightPose: Pose2d
    target_x_degrees: float
    target_y_degrees: float

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

        if self.limelightPose != None:
            self.limelightPose
            self.myField.setRobotPose(self.limelightPose)

        if self.pose:
            self.publishDouble("x", metersToFeet(self.pose.X()), "odom")
            self.publishDouble("y", metersToFeet(self.pose.Y()), "odom")
            self.publishDouble("angle", self.pose.rotation().degrees(), "odom")

    @classmethod
    def empty(cls, **kwargs):
        data = {f.name: None for f in fields(cls)}

        data.update(kwargs)

        return cls(**data)  # type: ignore
