import photonlibpy
from photonlibpy.photonCamera import PhotonCamera
from photonlibpy.photonPoseEstimator import PhotonPoseEstimator, PoseStrategy
from robotpy_apriltag import AprilTagField, AprilTagFieldLayout
from ntcore import NetworkTableInstance
import wpimath.geometry
import numpy
import robotpy
import wpilib




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
        self.photonTable = NetworkTableInstance.getDefault().getTable("photon")
        self.robotX: float = 0
        self.robotY: float = 0
        self.robotAngle: float = 0
        self.trustworthy = False
    def update(self):
        self.photonTable.putBoolean(self.cameraNameReal + " active", True)
        self.trustworthy = False
        self.camEstPose = None
        self.result = self.camera.getLatestResult()
        self.hasTargets = self.result.hasTargets()
        if self.hasTargets:
            self.target = self.result.getTargets()
            self.fiducialId = self.target[0].getFiducialId()
            self.ambiguity = self.target[0].getPoseAmbiguity()
            self.camEstPose = self.camPoseEst.update()
            if self.ambiguity < 0.04:
                self.trustworthy = True
                
                
                if self.camEstPose != None:
                    self.robotX = self.camEstPose.estimatedPose.X()
                    self.robotY = self.camEstPose.estimatedPose.Y()
                    self.robotZ = self.camEstPose.estimatedPose.Z()
                    self.robotAngle = self.camEstPose.estimatedPose.rotation().Z()
            else:
                pass
        else:
            self.ambiguity = 1
            self.fiducialId = -1
    
      

