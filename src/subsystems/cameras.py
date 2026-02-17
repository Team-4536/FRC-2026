import photonlibpy
from photonlibpy.photonCamera import PhotonCamera
from photonlibpy.photonPoseEstimator import PhotonPoseEstimator
from robotpy_apriltag import AprilTagField, AprilTagFieldLayout
from ntcore import NetworkTableInstance
import wpimath.geometry
import numpy
import robotpy
import wpilib
from photonlibpy import EstimatedRobotPose
from subsystems.subsystem import Subsystem
from subsystems.robotState import RobotState


class photonCameraClass:

    def __init__(self, cameraName, camPitch, intCamX, intCamY, intCamZ):
        self.cameraNameReal = cameraName

        self.camera = PhotonCamera(cameraName)
        kRobotToCam = wpimath.geometry.Transform3d(
            wpimath.geometry.Translation3d(intCamX, intCamY, intCamZ),
            wpimath.geometry.Rotation3d.fromDegrees(0.0, 0.0, camPitch),
        )
        self.camPoseEst = PhotonPoseEstimator(
            AprilTagFieldLayout.loadField(AprilTagField.k2026RebuiltWelded),
            kRobotToCam,
        )

        self.result = 0
        self.hasTargets = False
        self.target = 0
        self.ambiguity = 1

        self.robotX: float = 0
        self.robotY: float = 0
        self.robotAngle: float = 0
        self.trustworthy = False
        self.camEstPose: EstimatedRobotPose | None = None

    def update(self):

        self.trustworthy = False
        self.camEstPose = None
        self.result = self.camera.getLatestResult()
        self.hasTargets = self.result.hasTargets()
        if self.hasTargets:
            self.target = self.result.getTargets()
            self.fiducialId = self.target[0].getFiducialId()
            self.ambiguity = self.target[0].getPoseAmbiguity()

            if (
                self.ambiguity < 0.04
                and self.camPoseEst.estimateLowestAmbiguityPose(self.result)
                == EstimatedRobotPose
            ):
                self.trustworthy = True
                self.camEstPose = self.camPoseEst.estimateLowestAmbiguityPose(
                    self.result
                )
                if self.camEstPose != None:
                    self.camEstTrans = wpimath.geometry.Translation2d(
                        self.camEstPose.estimatedPose.X(),
                        self.camEstPose.estimatedPose.Y(),
                    )
                    self.camEstRot = wpimath.geometry.Rotation2d(
                        self.camEstPose.estimatedPose.rotation().Z()
                    )
                    self.camEstPose2d = wpimath.geometry.Pose2d(
                        self.camEstTrans, self.camEstRot
                    )
                    self.robotX = self.camEstPose.estimatedPose.X()
                    self.robotY = self.camEstPose.estimatedPose.Y()
                    self.robotZ = self.camEstPose.estimatedPose.Z()
                    self.robotAngle = self.camEstPose.estimatedPose.rotation().Z()
            else:
                pass
        else:
            self.ambiguity = 1
            self.fiducialId = -1


class CameraManager(Subsystem):
    def __init__(self):
        self.photonCameraRight = photonCameraClass(
            "Camera1",
            30,
            ((27 / 2) - (2 + 7 / 8)) * 0.0254,
            ((27 / 2) - 1.25) * 0.0254,
            ((8 + (3 / 8)) - 0.75) * 0.0254,
        )
        self.photonCameraLeft = photonCameraClass(
            "Camera2",
            -30,
            ((27 / 2) - 4.75) * 0.0254,
            ((27 / 2) - 1.25) * 0.0254,
            ((8 + (3 / 8)) - 0.75) * 0.0254,
        )
        self.photonCameraMiddle = photonCameraClass(
            "longCam",
            0,
            ((27 / 2) - (9 + (1 / 16))) * 0.0254,
            (27 / 2) * 0.0254,
            (10 + (7 / 8)) * 0.0254,
        )
        self.table = NetworkTableInstance.getDefault().getTable("telemetry")

    def phaseInit(self, robotState: RobotState):
        pass

    def periodic(self, robotState: RobotState) -> RobotState:
        self.table.putBoolean("camera periodic running", True)
        self.photonCameraRight.update()
        self.photonCameraLeft.update()
        self.photonCameraMiddle.update()
        if self.photonCameraLeft.trustworthy:
            robotState.odometry.addVisionMeasurement(
                self.photonCameraLeft.camEstPose2d, wpilib.getTime()
            )
        if self.photonCameraMiddle.trustworthy:
            robotState.odometry.addVisionMeasurement(
                self.photonCameraMiddle.camEstPose2d, wpilib.getTime()
            )

        if self.photonCameraRight.trustworthy:
            robotState.odometry.addVisionMeasurement(
                self.photonCameraRight.camEstPose2d, wpilib.getTime()
            )
        # self.publish()
        return robotState

    def disabled(self):
        pass

    def publish(self):

        self.table.putBoolean(
            "rightCam trustworthy", self.photonCameraRight.trustworthy
        )
        self.table.putBoolean("midCam trustworthy", self.photonCameraMiddle.trustworthy)
        self.table.putBoolean("leftCam trustworthy", self.photonCameraLeft.trustworthy)

        self.table.putNumber("midCamX", self.photonCameraMiddle.robotX)
        self.table.putNumber("midCamY", self.photonCameraMiddle.robotY)
        # self.table.putNumber("midCamZ", self.photonCameraMiddle.robotZ)
        self.table.putNumber("midCamRot", self.photonCameraMiddle.robotAngle)

        self.table.putNumber("rightCamX", self.photonCameraRight.robotX)
        self.table.putNumber("rightCamY", self.photonCameraRight.robotY)
        # self.table.putNumber("rightCamZ", self.photonCameraRight.robotZ)
        self.table.putNumber("rightCamRot", self.photonCameraRight.robotAngle)

        self.table.putNumber("leftCamX", self.photonCameraLeft.robotX)
        self.table.putNumber("leftCamY", self.photonCameraLeft.robotY)
        # self.table.putNumber("leftCamZ", self.photonCameraLeft.robotZ)
        self.table.putNumber("leftCamRot", self.photonCameraLeft.robotAngle)
