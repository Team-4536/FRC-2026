import wpilib
from ntcore import NetworkTableInstance, NetworkTable
from wpilib import DriverStation  # Talk to cremmet about robotState implementation
from wpimath.units import degreesToRadians
from wpimath.geometry import Pose2d, Rotation2d
from subsystems.subsystem import Subsystem
from subsystems.robotState import RobotState


class llCams(Subsystem):
    netTbl: NetworkTableInstance
    llTbl: NetworkTable

    # botposeWPIBLUE: any = [0,0,0,0,0,0,0,0,0,0,0]
    # botposeWPIRED: any = [0,0,0,0,0,0,0,0,0,0,0]

    botPoseWPI = [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]  # sets array to 0
    Team: DriverStation.Alliance = DriverStation.Alliance.kBlue  # selects driver side

    limelight2dPose: Pose2d

    def __init__(self) -> None:
        self.Team = (
            DriverStation.getAlliance() or DriverStation.Alliance.kBlue
        )  # sets DS or deafults to blue side

        self.netTbl = NetworkTableInstance.getDefault()
        self.llTbl = self.netTbl.getTable("limelight")

        self.botposeWPI = self.llTbl.getEntry("botpose_wpiblue").getDoubleArray(
            [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]
        )

        # if self.Team == DriverStation.Alliance.kBlue:
        #     self.botposeWPI = self.llTbl.getEntry("botpose_wpiblue").getDoubleArray(  # blue alliance code
        #         [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]
        # )
        # elif self.Team == DriverStation.Alliance.kRed:
        #     self.botposeWPI = self.llTbl.getEntry("botpose_wpired").getDoubleArray( # red alliance code
        #         [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]
        #     )
        # picks pose based on side

    def phaseInit(self, robotState: RobotState) -> RobotState:
        return robotState

    def periodic(self, robotState: RobotState) -> RobotState:
        # # botpose_wpiblue uses a 11 value array (index 0 = x, 2 = z, 5 = yaw)
        # if self.Team == DriverStation.Alliance.kBlue:
        #     self.botposeWPI = self.llTbl.getEntry("botpose_wpiblue").getDoubleArray( # blue alliance code
        #         [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]
        #     )
        # elif self.Team == DriverStation.Alliance.kRed:
        #     self.botposeWPI = self.llTbl.getEntry("botpose_wpibred").getDoubleArray(  # red alliance code
        #         [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]
        #     )

        llx = self.botposeWPI[0]  # sets x pose
        lly = self.botposeWPI[1]  # sets y pose

        yaw = degreesToRadians(self.botposeWPI[5])  # sets yaw (from degrees to radians)
        self.limelight2dPose = Pose2d(llx, lly, yaw)
        # robotState.limelightPose = self.limelight2dPoseRed
        if llx > 0 and lly > 0:
            robotState.limelightPose = self.limelight2dPose
            robotState.odometry.addVisionMeasurement(
                robotState.limelightPose, wpilib.getTime()
            )
            robotState.odometry.resetPose(robotState.odometry.getEstimatedPosition())
        else:
            robotState.limelightPose = None

        return robotState

    def disabled(self) -> None:
        pass

    def publish(self) -> None:
        self.netTbl.getTable("limelightDebug").getEntry("llx").setFloat(
            self.botposeWPI[0]
        )
        self.netTbl.getTable("limelightDebug").getEntry("lly").setFloat(
            self.botposeWPI[1]
        )
        self.netTbl.getTable("limelightDebug").getEntry("lly").setFloat(
            self.botposeWPI[5]
        )
        # robotState.myField.setRobotPose(self.limelightPose)


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
