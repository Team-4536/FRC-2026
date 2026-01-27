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
        self.camEstPose: EstimatedRobotPose = None

    def update(self):
       
        self.trustworthy = False
        self.camEstPose = None
        self.result = self.camera.getLatestResult()
        self.hasTargets = self.result.hasTargets()
        if self.hasTargets:
            self.target = self.result.getTargets()
            self.fiducialId = self.target[0].getFiducialId()
            self.ambiguity = self.target[0].getPoseAmbiguity()
            self.camEstPose = self.camPoseEst.estimateLowestAmbiguityPose()
            if self.ambiguity < 0.04 and self.camEstPose == EstimatedRobotPose:
                self.trustworthy = True
                self.robotX = self.camEstPose.estimatedPose.X()
                self.robotY = self.camEstPose.estimatedPose.Y()
                self.robotZ = self.camEstPose.estimatedPose.Z()
                self.robotAngle = self.camEstPose.estimatedPose.rotation().Z()
            else:
                pass
        else:
            self.ambiguity = 1
            self.fiducialId = -1
 
    
class photonCameraManager:
    def __init__(self):
        self.photonCameraRight = photonCameraClass("Camera1", 30, 0.28575, -0.028575, 1.0922)
        self.photonCameraLeft = photonCameraClass("Camera2", -30, 0.28575, 0.03175, 1.0922)
        self.photonCameraMiddle = photonCameraClass("longCam",0, 0.27305, -0.003175, 1.13665)
        self.table = NetworkTableInstance.getDefault().getTable("telemetry")
    def init(self):
        self.photonCameraRight = photonCameraClass("Camera1", 30, 0.28575, -0.028575, 1.0922)
        self.photonCameraLeft = photonCameraClass("Camera2", -30, 0.28575, 0.03175, 1.0922)
        self.photonCameraMiddle = photonCameraClass("longCam",0, 0.27305, -0.003175, 1.13665)
        self.table = NetworkTableInstance.getDefault().getTable("telemetry")
    def periodic(self):
        self.photonCameraRight.update()
        self.photonCameraLeft.update()
        self.photonCameraMiddle.update()
    def disabled(self):
        self.photonCameraRight.update()
        self.photonCameraLeft.update()
        self.photonCameraMiddle.update()
    def publish(self):
        
        self.table.putBoolean("rightCam trustworthy", self.photonCameraRight.trustworthy)
        self.table.putBoolean("midCam trustworthy", self.photonCameraMiddle.trustworthy)
        self.table.putBoolean("leftCam trustworthy", self.photonCameraLeft.trustworthy)

        self.table.putNumber("midCamX", self.photonCameraMiddle.robotX)
        self.table.putNumber("midCamY", self.photonCameraMiddle.robotY)
        self.table.putNumber("midCamRot", self.photonCameraMiddle.robotAngle)