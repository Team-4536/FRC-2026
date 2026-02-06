import limelight
import limelightresults
from ntcore import NetworkTableInstance
from subsystems.subsystem import Subsystem
from subsystems.robotState import RobotState


# import time
# import json
# from ntcore import NetworkTable
# from ntcore import NetworkTableInstance
# from networkTablesMixin import NetworkTablesMixin

# A = 0
# discovered_limelights = limelight.discover_limelights(debug=True)
# print("discovered limelights:", discovered_limelights)

# if discovered_limelights:
#     limelight_address = discovered_limelights[0]
#     ll = limelight.Limelight(limelight_address)
#     results = ll.get_results()
#     status = ll.get_status()
#     # print("-----")
#     # print("targeting results:", results)
#     # print("-----")
#     # print("status:", status)
#     # print("-----")
#     # print("temp:", ll.get_temp())
#     # print("-----")
#     # print("name:", ll.get_name())
#     # print("-----")
#     # print("fps:", ll.get_fps())
#     # print("-----")
#     # print("hwreport:", ll.hw_report())

#     ll.enable_websocket()

#     # print the current pipeline settings
#     # print(ll.get_pipeline_atindex(0))

#     try:
#         while A < 5:
#             result = ll.get_latest_results()
#             parsed_result = limelightresults.parse_results(result)
#             if parsed_result is not None:
#                 # print(
#                 #     "valid targets: ",
#                 #     parsed_result.validity,
#                 #     ", pipelineIndex: ",
#                 #     parsed_result.pipeline_id,
#                 #     ", Targeting Latency: ",
#                 #     parsed_result.targeting_latency,
#                 # )
#                 for tag in parsed_result.fiducialResults:
#                     print(tag.robot_pose_field_space, tag.fiducial_id)
#                     if tag.fiducial_id == 3:
#                         A += 1
#             time.sleep(1)  # Set this to 0 for max fps

#     except KeyboardInterrupt:
#         print("Program interrupted by user, shutting down.")
#     finally:
#         ll.disable_websocket()


# class FiducialResult:
#     def __init__(self, fiducial_data):
#         self.fiducial_id = fiducial_data["fID"] # prob want this
#         # self.family = fiducial_data["fam"]
#         # self.points = fiducial_data["pts"]
#         # self.skew = fiducial_data["skew"]
#         # self.camera_pose_target_space = fiducial_data["t6c_ts"]
#         # self.robot_pose_field_space = fiducial_data["t6r_fs"] # and this
#         # self.robot_pose_target_space = fiducial_data["t6r_ts"]
#         # self.target_area = fiducial_data["ta"]
#         self.target_x_degrees = fiducial_data["tx"] # |
#         self.target_x_pixels = fiducial_data["txp"] # These
#         self.target_y_degrees = fiducial_data["ty"] # Too
#         self.target_y_pixels = fiducial_data["typ"] # |

#     def init(self) -> None:
#         discovered_limelights = limelight.discover_limelights(debug=True)
#         print("discovered limelights:", discovered_limelights)

#         if discovered_limelights:
#             limelight_address = discovered_limelights[0]
#             self.ll = limelight.Limelight(limelight_address)
#             self.results = self.ll.get_results()
#             status = self.ll.get_status()
#             print("targeting results:", results)
#             print("status:", status)
#             print("name:", ll.get_name())
#             print("fps:", ll.get_fps())
#             print("hwreport:", ll.hw_report())

#             ll.enable_websocket()

#             # print(ll.get_pipeline_atindex(0))
#             limelightTable = NetworkTable.getSubTable("limelight")
#             print(self.fiducial_id)

#             # print(limelightTable)

#         print(self.target_x_degrees)
#         print(self.target_x_pixels)


class llCams(Subsystem):

    discovered_limelights: list = None
    ll: limelight = None

    def _init_(self) -> None:  # gets shown only when in robot.py
        self.llTable = NetworkTableInstance.getDefault().getTable("limelight")
        self.table = NetworkTableInstance.getDefault().getTable("telemetry")
        self.table.putNumber("limelight tx", self.llTable.getNumber("tx", 0))
        self.table.putNumber("limelight tx", self.llTable.getNumber("ty", 0))
        self.table.putNumber("limelight t6r_fs", self.llTable.getValue("t6r_fs", 0))

    def init(self) -> None:
        pass

    def periodic(self, robotState: RobotState) -> RobotState:
        self.llTable = NetworkTableInstance.getDefault().getTable("limelight")
        self.table = NetworkTableInstance.getDefault().getTable("telemetry")
        self.table.putNumber("limelight tx", self.llTable.getNumber("tx", 0))
        self.table.putNumber("limelight tx", self.llTable.getNumber("ty", 0))
        self.table.putNumber("limelight t6r_fs", self.llTable.getValue("t6r_fs", 0))

    def disabled(self) -> None:
        pass

    def publish(self) -> None:
        pass


# X and Y are converted coordinates from inches to meters
aprilTagX = {
    11.8781,
    11.9155,
    11.3119,
    11.3119,
    11.9155,
    11.8781,
    11.9530,
    12.2710,
    12.5191,
    12.5191,
    12.2710,
    11.9530,
    16.5334,
    16.5334,
    16.5329,
    16.5329,
    4.6632,
    4.6257,
    5.2291,
    5.2291,
    4.6257,
    4.6632,
    4.5883,
    4.2699,
    4.0218,
    4.2018,
    4.2699,
    4.5883,
    0.0076,
    0.0076,
    0.0081,
    0.0081,
}

aprilTagY = {
    7.4247,
    4.6380,
    4.3801,
    4.0345,
    3.4313,
    0.6444,
    0.6444,
    3.4313,
    3.6789,
    4.0345,
    463.80,
    7.4247,
    7.4034,
    6.9715,
    4.3236,
    3.8918,
    0.6444,
    3.4313,
    3.6789,
    4.0345,
    4.6380,
    7.4247,
    7.4247,
    4.6380,
    4.2801,
    4.0345,
    3.4313,
    0.6444,
    0.6660,
    1.0978,
    3.7457,
    4.1776,
}

aprilTagDegrees = {
    180,
    90,
    180,
    180,
    270,
    180,
    0,
    270,
    0,
    0,
    90,
    0,
    180,
    180,
    180,
    180,
    0,
    270,
    0,
    0,
    90,
    0,
    180,
    90,
    180,
    180,
    270,
    180,
    0,
    0,
    0,
    0,
}
