from photonlibpy.photonCamera import PhotonCamera
from photonlibpy.photonPoseEstimator import PhotonPoseEstimator
from robotpy_apriltag import AprilTagField, AprilTagFieldLayout

import wpimath.geometry

from ntcore import NetworkTableInstance
from photonlibpy import EstimatedRobotPose
from subsystems.networkTablesMixin import NetworkTablesMixin
from subsystems.robotState import RobotState
from subsystems.subsystem import Subsystem
from wpimath.units import inchesToMeters
from wpilib import getTime


class photonCameraClass(NetworkTablesMixin):

    def __init__(
        self,
        cameraName: str,
        camPitch: float,
        camYaw: float,
        intCamX: float,
        intCamY: float,
        intCamZ: float,
    ):
        super().__init__()
        self.cameraNameReal = cameraName

        self.camera = PhotonCamera(cameraName)
        kRobotToCam = wpimath.geometry.Transform3d(
            wpimath.geometry.Translation3d(intCamX, intCamY, intCamZ),
            wpimath.geometry.Rotation3d.fromDegrees(0.0, camPitch, camYaw),
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
        self.hasTargetsRan = False
        self.table = NetworkTableInstance.getDefault().getTable("telemetry")

    def update(self):
        self.hasTargetsRan = False
        self.trustworthy = False
        self.camEstPose = None
        self.result = self.camera.getLatestResult()
        self.hasTargets = self.result.hasTargets()

        if self.hasTargets:
            self.running = True
            self.hasTargetsRan = True
            self.target = self.result.getTargets()
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

                    self.robotAngle = self.camEstPose.estimatedPose.rotation().Z()

                    # self.publishFloat("robotXodom", self.robotX.real)
                    # self.publishFloat("robotYodom", self.robotY.real)
                    # self.publishFloat("robotZodom", self.robotAngle.real)
                    self.table.putNumber("camOdomX", self.robotX)
                    self.table.putNumber("camOdomY", self.robotY)
                    self.table.putNumber("camOdomR", self.robotAngle)
                elif (
                    len(self.target) > 1
                    and type(self.camPoseEst.estimateCoprocMultiTagPose(self.result))
                    == EstimatedRobotPose
                ):
                    self.trustworthy = True
                self.camEstPose = self.camPoseEst.estimateCoprocMultiTagPose(
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

                    self.robotAngle = self.camEstPose.estimatedPose.rotation().Z()

                    # self.publishFloat("robotXodom", self.robotX.real)
                    # self.publishFloat("robotYodom", self.robotY.real)
                    # self.publishFloat("robotZodom", self.robotAngle.real)
                    self.table.putNumber("camOdomX", self.robotX)
                    self.table.putNumber("camOdomY", self.robotY)
                    self.table.putNumber("camOdomR", self.robotAngle)

            else:
                pass
        else:
            self.ambiguity = 1
            self.fiducialId = -1
            self.running = False


class CameraManager(Subsystem):
    def __init__(self):

        super().__init__()
        self.photonCameraRight = photonCameraClass(
            "Camera1",
            15,
            -30,
            inchesToMeters(27 / 2) - (9.1 / 100),
            inchesToMeters(27 / 2) - (4.4 / 100),
            27.5 / 100,
        )
        self.photonCameraLeft = photonCameraClass(
            "Camera2",
            15,
            30,
            inchesToMeters(27 / 2) - (15.3 / 100),
            inchesToMeters(27 / 2) - (4.4 / 100),
            27.5 / 100,
        )
        # self.photonCameraMiddle = photonCameraClass(
        #     "longCam", strip.show();
        #     0,
        #     ((27 / 2) - (4 + 3 / 8)) * 0.0254,
        #     ((27 / 2) - 1 + 1 / 8) * 0.0254,
        #     (10.5 + 26.5) * 0.0254,
        # )

    def phaseInit(self, robotState: RobotState) -> RobotState:
        return robotState

    def periodic(self, robotState: RobotState) -> RobotState:

        self.photonCameraRight.update()
        self.photonCameraLeft.update()
        self.publishBoolean("cam1 running", self.photonCameraRight.running)

        self.publishBoolean("cam2 running", self.photonCameraLeft.running)

        # self.photonCameraMiddle.update()

        if self.photonCameraLeft.trustworthy:
            robotState.odometry.addVisionMeasurement(
                self.photonCameraLeft.camEstPose2d, getTime()
            )
        # if self.photonCameraMiddle.trustworthy:

        #     robotState.odometry.addVisionMeasurement(
        #         self.photonCameraMiddle.camEstPose2d, getTime()
        #     )

        if self.photonCameraRight.trustworthy:
            robotState.odometry.addVisionMeasurement(
                self.photonCameraRight.camEstPose2d, getTime()
            )
        self.publish()
        # self.a = wpimath.geometry.Pose2d(5, 5, 12039)
        # robotState.odometry.addVisionMeasurement(self.a, getTime())

        robotState.odometry.resetPose(robotState.odometry.getEstimatedPosition())

        # resetPosition(
        #         self._gyro.getRotation2d(),
        #         self._modules.modulePositions,
        #         Pose2d(
        #             robotState.odometry.getEstimatedPosition().translation(),
        #             Rotation2d(),
        #         ),
        #     )
        # robotState.odometry.resetPosition
        return robotState

    def disabled(self):
        pass

    def publish(self):

        self.publishBoolean("rightCam trustworthy", self.photonCameraRight.trustworthy)
        # self.publishBoolean("midCam trustworthy", self.photonCameraMiddle.trustworthy)
        self.publishBoolean("leftCam trustworthy", self.photonCameraLeft.trustworthy)
        self.publishFloat("leftCamAmbiguity", self.photonCameraLeft.ambiguity)
        self.publishFloat("rightCamAmbiguity", self.photonCameraRight.ambiguity)

        # self.publishFloat("midCamX", self.photonCameraMiddle.robotX)
        # self.publishFloat("midCamY", self.photonCameraMiddle.robotY)
        # self.publishFloat("midCamRot", self.photonCameraMiddle.robotAngle)
        self.publishFloat("rightCamX", self.photonCameraRight.robotX)
        self.publishFloat("rightCamY", self.photonCameraRight.robotY)
        self.publishFloat("rightCamRot", self.photonCameraRight.robotAngle)
        self.publishFloat("leftCamX", self.photonCameraLeft.robotX)
        self.publishFloat("leftCamY", self.photonCameraLeft.robotY)
        self.publishFloat("leftCamRot", self.photonCameraLeft.robotAngle)
