import wpilib
from cameras import photonCameraClass
from ntcore import NetworkTableInstance

class Robot(wpilib.TimedRobot):
    def robotInit(self):
        self.photonCamera1 = photonCameraClass("Camera1", 0, 0, 0, 0)
        self.photonCamera2 = photonCameraClass("Camera2", 0, 0, 0, 0)
        self.currX = 0
        self.currY = 0
        self.currRot = 0
        self.table = NetworkTableInstance().getDefault().getTable("telemetry")
    def robotPeriodic(self):
        if(self.photonCamera1.trustworthy and self.photonCamera2.trustworthy):
            self.currX = (self.photonCamera1.robotX + self.photonCamera2.robotX) / 2
            self.currY = (self.photonCamera1.robotY + self.photonCamera2.robotY) / 2
            self.currRot = (self.photonCamera1.robotAngle + self.photonCamera2.robotAngle) / 2
        elif(self.photonCamera1.trustworthy):
            self.currX = self.photonCamera1.robotX
            self.currY = self.photonCamera1.robotY
            self.currRot = self.photonCamera1.robotAngle
        elif(self.photonCamera2.trustworthy):
            self.currX = self.photonCamera2.robotX
            self.currY = self.photonCamera2.robotY
            self.currRot = self.photonCamera2.robotAngle
        else:
            self.currX = 0
            self.currY = 0
            self.currRot = 0
        self.table.putNumber("x", self.currX)
        self.table.putNumber("y", self.currY)
        self.table.putNumber("rotation", self.currRot)
    def teleopInit(self):
        pass
    def teleopPeriodic(self):
        pass