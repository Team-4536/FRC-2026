from photonlibpy.photonCamera import PhotonCamera
from photonlibpy.photonPoseEstimator import PhotonPoseEstimator
from robotpy_apriltag import AprilTagField, AprilTagFieldLayout

import wpimath.geometry

import wpilib
from photonlibpy import EstimatedRobotPose
from subsystems.subsystem import Subsystem
from subsystems.robotState import RobotState


class photonCameraClass:

    def __init__(
        self,
        cameraName: str,
        camPitch: float,
        intCamX: float,
        intCamY: float,
        intCamZ: float,
    ):
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
        self.hasTargetsRan = False

    def update(self):
        self.hasTargetsRan = False
        self.trustworthy = False
        self.camEstPose = None
        self.result = self.camera.getLatestResult()
        self.hasTargets = self.result.hasTargets()

        if self.hasTargets:

            self.hasTargetsRan = True
            self.target = self.result.getTargets()
            self.fiducialId = self.target[0].getFiducialId()
            self.ambiguity = self.target[0].getPoseAmbiguity()

            if (
                self.ambiguity < 0.04
                and type(self.camPoseEst.estimateLowestAmbiguityPose(self.result))
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

        super().__init__()
        self.photonCameraRight = photonCameraClass(
            "Camera1",
            30,
            ((27 / 2) - (3 + 5 / 16)) * 0.0254,
            ((27 / 2) - 1) * 0.0254,
            (8 + 5 / 16 + 26.5) * 0.0254,
        )
        self.photonCameraLeft = photonCameraClass(
            "Camera2",
            -30,
            ((27 / 2) - (5 + 5 / 8)) * 0.0254,
            ((27 / 2) - 1) * 0.0254,
            (8 + 5 / 16 + 26.5) * 0.0254,
        )
        self.photonCameraMiddle = photonCameraClass(
            "longCam",
            0,
            ((27 / 2) - (4 + 3 / 8)) * 0.0254,
            ((27 / 2) - 1 + 1 / 8) * 0.0254,
            (10.5 + 26.5) * 0.0254,
        )

    def phaseInit(self, robotState: RobotState) -> RobotState:
        return robotState

    def periodic(self, robotState: RobotState) -> RobotState:

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

        self.publishBoolean("rightCam trustworthy", self.photonCameraRight.trustworthy)
        self.publishBoolean("midCam trustworthy", self.photonCameraMiddle.trustworthy)
        self.publishBoolean("leftCam trustworthy", self.photonCameraLeft.trustworthy)
        self.publishFloat("midCamX", self.photonCameraMiddle.robotX)
        self.publishFloat("midCamY", self.photonCameraMiddle.robotY)
        self.publishFloat("midCamRot", self.photonCameraMiddle.robotAngle)
        self.publishFloat("rightCamX", self.photonCameraRight.robotX)
        self.publishFloat("rightCamY", self.photonCameraRight.robotY)
        self.publishFloat("rightCamRot", self.photonCameraRight.robotAngle)
        self.publishFloat("leftCamX", self.photonCameraLeft.robotX)
        self.publishFloat("leftCamY", self.photonCameraLeft.robotY)
        self.publishFloat("leftCamRot", self.photonCameraLeft.robotAngle)
