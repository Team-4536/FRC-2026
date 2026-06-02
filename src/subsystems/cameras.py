from ntcore import NetworkTableInstance
from photonlibpy import EstimatedRobotPose
from photonlibpy.photonCamera import PhotonCamera
from photonlibpy.photonPoseEstimator import PhotonPoseEstimator
from robotpy_apriltag import AprilTagField, AprilTagFieldLayout
from subsystems.networkTablesMixin import NetworkTablesMixin
from subsystems.robotState import RobotState
from subsystems.subsystem import Subsystem
from wpimath.geometry import (
    Pose2d,
    Rotation2d,
    Rotation3d,
    Transform3d,
    Translation2d,
    Translation3d,
)
from wpimath.units import inchesToMeters, radiansToDegrees


class photonCameraClass(NetworkTablesMixin):
    def __init__(
        self,
        cameraName: str,
        camPitch: float,
        camYaw: float,
        intCamX: float,
        intCamY: float,
        intCamZ: float,
    ) -> (
        None
    ):  # All of those values are relative to the robot using the robot coordinate system, using meters and degrees
        super().__init__()

        self.camera = PhotonCamera(
            cameraName
        )  # This has to exactly match with the name of the camera in the photon client
        kRobotToCam = Transform3d(
            Translation3d(intCamX, intCamY, intCamZ),
            Rotation3d.fromDegrees(0.0, camPitch, camYaw),
        )
        self.camPoseEst = PhotonPoseEstimator(
            AprilTagFieldLayout.loadField(AprilTagField.k2026RebuiltWelded),
            kRobotToCam,
        )

        self.result = 0
        self.hasTargets = False
        self.target = [-1, -1]
        self.ambiguity = 1

        self.robotX: float = 0
        self.robotY: float = 0
        self.robotAngle: float = 0
        self.trustworthy = False
        self.camEstPose: EstimatedRobotPose | None = None

        self.table = NetworkTableInstance.getDefault().getTable("telemetry")
        self.timeStamp = -1

    def update(self) -> None:

        self.trustworthy = False
        self.camEstPose = None
        self.result = self.camera.getLatestResult()
        self.hasTargets = (
            self.result.hasTargets()
        )  # Checks if the latest camera result has an april tag in sight

        if self.hasTargets:

            self.target = (
                self.result.getTargets()
            )  # Gets list of all april tags in view
            self.fiducialId = self.target[0].getFiducialId()
            self.ambiguity = self.target[0].getPoseAmbiguity()

            if (
                self.ambiguity < 0.15
                and type(self.camPoseEst.estimateLowestAmbiguityPose(self.result))
                == EstimatedRobotPose
            ):

                self.trustworthy = True
                self.camEstPose = self.camPoseEst.estimateLowestAmbiguityPose(
                    self.result
                )
                if self.camEstPose != None:
                    self.camEstTrans = Translation2d(
                        self.camEstPose.estimatedPose.X(),
                        self.camEstPose.estimatedPose.Y(),
                    )
                    self.camEstRot = Rotation2d(
                        self.camEstPose.estimatedPose.rotation().Z()
                    )
                    self.camEstPose2d = Pose2d(self.camEstTrans, self.camEstRot)
                    self.timeStamp = self.camEstPose.timestampSeconds
                    self.robotX = self.camEstPose.estimatedPose.X()
                    self.robotY = self.camEstPose.estimatedPose.Y()

                    self.robotAngle = self.camEstPose.estimatedPose.rotation().Z()
                elif (
                    len(self.target) > 1
                    and type(self.camPoseEst.estimateCoprocMultiTagPose(self.result))
                    == EstimatedRobotPose
                    and self.target[0].getPoseAmbiguity() < 0.16
                    and self.target[1].getPoseAmbiguity() < 0.16
                ):
                    self.trustworthy = True
                self.camEstPose = self.camPoseEst.estimateCoprocMultiTagPose(
                    self.result
                )
                if self.camEstPose != None:
                    self.camEstTrans = Translation2d(
                        self.camEstPose.estimatedPose.X(),
                        self.camEstPose.estimatedPose.Y(),
                    )
                    self.camEstRot = Rotation2d(
                        self.camEstPose.estimatedPose.rotation().Z()
                    )
                    self.camEstPose2d = Pose2d(self.camEstTrans, self.camEstRot)
                    self.timeStamp = self.camEstPose.timestampSeconds
                    self.robotX = self.camEstPose.estimatedPose.X()
                    self.robotY = self.camEstPose.estimatedPose.Y()

                    self.robotAngle = self.camEstPose.estimatedPose.rotation().Z()
            else:
                pass
        else:
            self.ambiguity = 1
            self.fiducialId = -1


class CameraManager(Subsystem):
    def __init__(self) -> None:
        super().__init__()

        self.photonCameraRight = photonCameraClass(
            "Camera1",  # Name the cameras better than this, minutes wasted checking camera names: 30 <- that is an underestimate
            15,
            -30,
            inchesToMeters(27 / 2) - (9 / 100),
            -(inchesToMeters(27 / 2) - (6.6 / 100)),
            (25.4 + 3.9) / 100 + inchesToMeters(0.5),
        )
        self.photonCameraLeft = photonCameraClass(
            "Camera2",
            15,
            30 - radiansToDegrees(0.1),
            inchesToMeters(27 / 2) - (9 / 100),
            -(inchesToMeters(27 / 2) - (12.5 / 100)),
            (25.4 + 3.9) / 100 + inchesToMeters(0.5),
        )

    def phaseInit(self, robotState: RobotState) -> None:
        pass

    def periodic(self, robotState: RobotState) -> None:
        self.photonCameraRight.update()
        self.photonCameraLeft.update()

        if self.photonCameraLeft.trustworthy:

            robotState.odometry.addVisionMeasurement(
                self.photonCameraLeft.camEstPose2d,
                self.photonCameraLeft.timeStamp,
            )

        if self.photonCameraRight.trustworthy:
            robotState.odometry.addVisionMeasurement(
                self.photonCameraRight.camEstPose2d,
                self.photonCameraRight.timeStamp,
            )

        robotState.odometry.resetPose(
            robotState.odometry.getEstimatedPosition()
        )  # I have no idea why this line is necessary

    def disabled(self) -> None:
        pass

    def publish(self) -> None:
        self.publishBoolean("rightCam trustworthy", self.photonCameraRight.trustworthy)

        self.publishBoolean("leftCam trustworthy", self.photonCameraLeft.trustworthy)
        self.publishFloat("leftCamAmbiguity", self.photonCameraLeft.ambiguity)
        self.publishFloat("rightCamAmbiguity", self.photonCameraRight.ambiguity)

        self.publishFloat("rightCamX", self.photonCameraRight.robotX)
        self.publishFloat("rightCamY", self.photonCameraRight.robotY)
        self.publishFloat("rightCamRot", self.photonCameraRight.robotAngle)
        self.publishFloat("leftCamX", self.photonCameraLeft.robotX)
        self.publishFloat("leftCamY", self.photonCameraLeft.robotY)
        self.publishFloat("leftCamRot", self.photonCameraLeft.robotAngle)
