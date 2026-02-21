import limelight
import limelightresults
from wpilib import Field2d
from ntcore import NetworkTableInstance, NetworkTable
from wpimath.units import degreesToRadians
from wpimath.geometry import Pose2d, Rotation2d
from subsystems.subsystem import Subsystem
from subsystems.robotState import RobotState


class llCams(Subsystem):

    def __init__(self) -> None:
        self.llTable = (
            NetworkTableInstance.getDefault()
            .getTable("limelight")
            .getEntry("botpose_wpiblue")
        )
        self.table = NetworkTableInstance.getDefault().getTable("telemetry")

    def init(self) -> None:
        pass

    def periodic(self, robotState: RobotState) -> RobotState:
        # botpose_wpiblue 4 is x, 5 is y, 9 is yaw
        self.llTable
        robotState.limelightPose = Pose2d(
            self.llTable.getDoubleArray(4),
            self.llTable.getDoubleArray(5),
            Rotation2d(degreesToRadians(self.llTable.getDoubleArray(9))),
        )
        return robotState

    def disabled(self) -> None:
        pass

    def publish(self) -> None:
        pass


# # X and Y are converted coordinates from inches to meters
# aprilTagX = {
#     11.8781,
#     11.9155,
#     11.3119,
#     11.3119,
#     11.9155,
#     11.8781,
#     11.9530,
#     12.2710,
#     12.5191,
#     12.5191,
#     12.2710,
#     11.9530,
#     16.5334,
#     16.5334,
#     16.5329,
#     16.5329,
#     4.6632,
#     4.6257,
#     5.2291,
#     5.2291,
#     4.6257,
#     4.6632,
#     4.5883,
#     4.2699,
#     4.0218,
#     4.2018,
#     4.2699,
#     4.5883,
#     0.0076,
#     0.0076,
#     0.0081,
#     0.0081,
# }
# # X and Y are converted coordinates from inches to meters
# aprilTagX = {
#     11.8781,
#     11.9155,
#     11.3119,
#     11.3119,
#     11.9155,
#     11.8781,
#     11.9530,
#     12.2710,
#     12.5191,
#     12.5191,
#     12.2710,
#     11.9530,
#     16.5334,
#     16.5334,
#     16.5329,
#     16.5329,
#     4.6632,
#     4.6257,
#     5.2291,
#     5.2291,
#     4.6257,
#     4.6632,
#     4.5883,
#     4.2699,
#     4.0218,
#     4.2018,
#     4.2699,
#     4.5883,
#     0.0076,
#     0.0076,
#     0.0081,
#     0.0081,
# }

# aprilTagY = {
#     7.4247,
#     4.6380,
#     4.3801,
#     4.0345,
#     3.4313,
#     0.6444,
#     0.6444,
#     3.4313,
#     3.6789,
#     4.0345,
#     463.80,
#     7.4247,
#     7.4034,
#     6.9715,
#     4.3236,
#     3.8918,
#     0.6444,
#     3.4313,
#     3.6789,
#     4.0345,
#     4.6380,
#     7.4247,
#     7.4247,
#     4.6380,
#     4.2801,
#     4.0345,
#     3.4313,
#     0.6444,
#     0.6660,
#     1.0978,
#     3.7457,
#     4.1776,
# }

# aprilTagDegrees = {
#     180,
#     90,
#     180,
#     180,
#     270,
#     180,
#     0,
#     270,
#     0,
#     0,
#     90,
#     0,
#     180,
#     180,
#     180,
#     180,
#     0,
#     270,
#     0,
#     0,
#     90,
#     0,
#     180,
#     90,
#     180,
#     180,
#     270,
#     180,
#     0,
#     0,
#     0,
#     0,
# }
