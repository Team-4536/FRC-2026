import limelight
import limelightresults
import time
from desiredState import DesiredState
from subsystem import Subsystem

# import json
# import ntcore
# from ntcore import NetworkTableInstance
from ntcore import NetworkTable


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
#         self.fiducial_id = fiducial_data["fID"]
#         # self.family = fiducial_data["fam"]
#         # self.points = fiducial_data["pts"]
#         # self.skew = fiducial_data["skew"]
#         # self.camera_pose_target_space = fiducial_data["t6c_ts"]
#         # self.robot_pose_field_space = fiducial_data["t6r_fs"]
#         # self.robot_pose_target_space = fiducial_data["t6r_ts"]
#         # self.target_area = fiducial_data["ta"]
#         self.target_x_degrees = fiducial_data["tx"]
#         self.target_x_pixels = fiducial_data["txp"]
#         self.target_y_degrees = fiducial_data["ty"]
#         self.target_y_pixels = fiducial_data["typ"]
#         pass

#     # aprilTagX = {}

#     # aprilTagY = {}
#     def init(self) -> None:
#         discovered_limelights = limelight.discover_limelights(debug=True)
#         print("discovered limelights:", discovered_limelights)

#         if discovered_limelights:
#             limelight_address = discovered_limelights[0]
#             self.ll = limelight.Limelight(limelight_address)
#             self.results = self.ll.get_results()
#             status = self.ll.get_status()
#             print("-----")
#             print("targeting results:", results)
#             print("-----")
#             print("status:", status)
#             print("-----")
#             print("temp:", ll.get_temp())
#             print("-----")
#             print("name:", ll.get_name())
#             print("-----")
#             print("fps:", ll.get_fps())
#             print("-----")
#             print("hwreport:", ll.hw_report())

#             ll.enable_websocket()

#             # print the current pipeline settings

#             # print(ll.get_pipeline_atindex(0))  # Required
#             limelightTable = NetworkTable.getSubTable("limelight")
#             print(self.fiducial_id)

#             # print(limelightTable)

#     def periodic(self) -> None:
#         print("Hello World")
#         print(self.target_x_degrees)
#         print(self.target_x_pixels)

#     # self.ll.

#     def disable(self) -> None:
#         ll.disable_websocket()
#         pass

#     def publish(self) -> None:
#         pass


class llCams(Subsystem):

    discovered_limelights: list = None
    ll: limelight = None

    def _init_(self) -> None:
        pass

    def init(self) -> None:
        self.discovered_limelights = limelight.discover_limelights(debug=True)
        print("cam count:", len(self.discovered_limelights))
        limelight_address = self.discovered_limelights[0]
        self.ll = limelight.Limelight(limelight_address)
        print("discovered limelights:", self.discovered_limelights)
        self.ll.enable_websocket()

    def periodic(self, ds: DesiredState) -> None:
        if self.discovered_limelights and self.ll:
            result = self.ll.get_latest_results()
            parsed_result = limelightresults.parse_results(result)
            for tag in parsed_result.fiducialResults:
                print(tag.robot_pose_field_space, tag.fiducial_id)

    def disabled(self) -> None:
        if self.ll:
            self.ll.disable_websocket()

    def publish(self) -> None:
        pass


ds: DesiredState = DesiredState(0, 0)
llcam = llCams()
llcam.init()
llcam.periodic(ds)
llcam.disabled()
