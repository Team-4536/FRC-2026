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
            
            if self.ambiguity < 0.04 and self.camPoseEst.estimateLowestAmbiguityPose() == EstimatedRobotPose:
                self.trustworthy = True
                self.camEstPose = self.camPoseEst.estimateLowestAmbiguityPose()
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
        self.camXList = []
        self.camYList = []
        self.camRotList = []
        self.aveX = -1
        self.aveY = -1
        self.aveRot = -1

    def init(self):
        self.photonCameraRight = photonCameraClass("Camera1", 30, 0.28575, -0.028575, 1.0922)
        self.photonCameraLeft = photonCameraClass("Camera2", -30, 0.28575, 0.03175, 1.0922)
        self.photonCameraMiddle = photonCameraClass("longCam",0, 0.27305, -0.003175, 1.13665)
        self.table = NetworkTableInstance.getDefault().getTable("telemetry")
        self.camXList = []
        self.camYList = []
        self.camRotList = []
        self.camZList = []
        self.aveX = -1
        self.aveY = -1
        self.aveRot = -1
        self.aveZ = -1
        self.aveTranslation3d = wpimath.geometry.Translation3d(0,0,0)
        self.aveRotation3d = wpimath.geometry.Rotation3d(0,0,0)
        
        self.avePose = wpimath.geometry.Pose3d(self.aveTranslation3d, self.aveRotation3d)
    def periodic(self):
        self.avePose = None
        self.camXList = []
        self.camYList = []
        self.camZList = []
        self.camRotList = []
        self.aveX = -1
        self.aveY = -1
        self.aveRot = -1
        self.aveZ = -1
        self.photonCameraRight.update()
        self.photonCameraLeft.update()
        self.photonCameraMiddle.update()
        if self.photonCameraLeft.trustworthy:
            self.camXList.append(self.photonCameraLeft.robotX)
            self.camYList.append(self.photonCameraLeft.robotY)
            self.camRotList.append(self.photonCameraLeft.robotAngle)
            self.camZList.append(self.photonCameraLeft.robotZ)
        if self.photonCameraMiddle.trustworthy:
            self.camXList.append(self.photonCameraMiddle.robotX)
            self.camYList.append(self.photonCameraMiddle.robotY)
            self.camRotList.append(self.photonCameraMiddle.robotAngle)
            self.camZList.append(self.photonCameraMiddle.robotZ)
        if self.photonCameraRight.trustworthy:
            self.camXList.append(self.photonCameraRight.robotX)
            self.camYList.append(self.photonCameraRight.robotY)
            self.camRotList.append(self.photonCameraRight.robotAngle)
            self.camZList.append(self.photonCameraRight.robotZ)
        if(len(self.camXList) > 0):
            self.aveX = self.average(self.camXList)
            self.aveY = self.average(self.camYList)
            self.aveZ = self.average(self.camZList)
            self.aveRot = self.average(self.camRotList)
        if(not (self.aveX == -1 and self.aveY == -1 and self.aveZ == -1)):
            self.aveTranslation3d = wpimath.geometry.Translation3d(self.aveX, self.aveY, self.aveZ)
            self.aveRotation3d = wpimath.geometry.Rotation3d(self.aveRot,0 ,0)
            self.avePose = wpimath.geometry.Pose3d(self.aveTranslation3d, self.aveRotation3d)

    def disabled(self):
        self.periodic()

    def publish(self):

        self.table.putBoolean("rightCam trustworthy", self.photonCameraRight.trustworthy)
        self.table.putBoolean("midCam trustworthy", self.photonCameraMiddle.trustworthy)
        self.table.putBoolean("leftCam trustworthy", self.photonCameraLeft.trustworthy)

        self.table.putNumber("midCamX", self.photonCameraMiddle.robotX)
        self.table.putNumber("midCamY", self.photonCameraMiddle.robotY)
        self.table.putNumber("midCamZ", self.photonCameraMiddle.robotZ)
        self.table.putNumber("midCamRot", self.photonCameraMiddle.robotAngle)

        self.table.putNumber("rightCamX", self.photonCameraRight.robotX)
        self.table.putNumber("rightCamY", self.photonCameraRight.robotY)
        self.table.putNumber("rightCamZ", self.photonCameraRight.robotZ)
        self.table.putNumber("rightCamRot", self.photonCameraRight.robotAngle)

        self.table.putNumber("leftCamX", self.photonCameraLeft.robotX)
        self.table.putNumber("leftCamY", self.photonCameraLeft.robotY)
        self.table.putNumber("leftCamZ", self.photonCameraLeft.robotZ)
        self.table.putNumber("leftCamRot", self.photonCameraLeft.robotAngle)

    def average(vals: list, ) -> int:
        for i in vals:
            temp += vals[i]
        temp = temp / len(vals)
        return temp