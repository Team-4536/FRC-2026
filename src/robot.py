import wpilib
import wpimath
from cameras import photonCameraClass
from ntcore import NetworkTableInstance
from rev import (
    SparkMax,
    SparkMaxConfig,
    SparkClosedLoopController,
    ClosedLoopConfig,
    ClosedLoopSlot,
    LimitSwitchConfig,
)
import math

class Robot(wpilib.TimedRobot):
    def robotInit(self):
        self.whereToAimRot = wpimath.geometry.Rotation3d
        self.whereToAim = wpimath.geometry.Pose3d(wpimath.geometry.Transform3d)
        
        self.photonCameraRight = photonCameraClass("Camera1", 30, 0.28575, -0.028575, 1.270127)
        self.photonCameraLeft = photonCameraClass("Camera2", -30, 0.28575, 0.03175, 1.270127)
        self.photonCameraMiddle = photonCameraClass("longCam",0, 0.27305, -0.003175, 1.314577)
        self.currX = 0
        self.currY = 0
        self.currRot = 0
        self.currZ = 0
        self.table = NetworkTableInstance.getDefault().getTable("telemetry")
        self.tagX = 11.56081
        self.tagY = 8.055
        self.count = 1
        self.tempX = 0
        self.tempY = 0
        self.tempRot = 0
        self.driveCtrlr = wpilib.XboxController(0)
        self.turretMotor = SparkMax(3, SparkMax.MotorType.kBrushless)
        self.desiredAngle = 0
    def robotPeriodic(self):
        self.photonCameraRight.update()
        self.photonCameraLeft.update()
        self.photonCameraMiddle.update()
        self.table.putBoolean("rightCam trustworthy", self.photonCameraRight.trustworthy)
        self.table.putBoolean("midCam trustworthy", self.photonCameraMiddle.trustworthy)
        self.table.putBoolean("leftCam trustworthy", self.photonCameraLeft.trustworthy)
        if(self.photonCameraRight.trustworthy and self.photonCameraLeft.trustworthy):
            self.currX = (self.photonCameraRight.robotX + self.photonCameraLeft.robotX) / 2
            self.currY = (self.photonCameraRight.robotY + self.photonCameraLeft.robotY) / 2
            self.currRot = (self.photonCameraRight.robotAngle + self.photonCameraLeft.robotAngle) / 2
        elif(self.photonCameraRight.trustworthy):
            self.currX = self.photonCameraRight.robotX
            self.currY = self.photonCameraRight.robotY
            self.currRot = self.photonCameraRight.robotAngle
        elif(self.photonCameraLeft.trustworthy):
            self.currX = self.photonCameraLeft.robotX
            self.currY = self.photonCameraLeft.robotY
            self.currRot = self.photonCameraLeft.robotAngle
            self.currZ = self.photonCameraLeft.robotZ
        if(self.photonCameraLeft.trustworthy):
            self.tempX += self.photonCameraLeft.robotX
            self.tempY += self.photonCameraLeft.robotY
            self.tempRot += self.photonCameraLeft.robotAngle
            self.count += 1
        if(self.photonCameraMiddle.trustworthy):
            self.tempX += self.photonCameraMiddle.robotX
            self.tempY += self.photonCameraMiddle.robotY
            self.tempRot += self.photonCameraMiddle.robotAngle
            self.count += 1
        if(self.photonCameraRight.trustworthy):
            self.tempX += self.photonCameraRight.robotX
            self.tempY += self.photonCameraRight.robotY
            self.tempRot += self.photonCameraRight.robotAngle
            self.count += 1
        if(self.tempX >= 0 and self.tempY >= 0):
            self.currX = self.tempX / self.count
            self.currY = self.tempY / self.count
            self.currRot = self.tempRot / self.count
        self.table.putBoolean("cam1 trustworthy", self.photonCameraRight.trustworthy)
        self.table.putBoolean("cam2 trustworthy", self.photonCameraLeft.trustworthy)
        self.table.putNumber("x", self.currX)
        self.table.putNumber("y", self.currY)
        self.table.putNumber("adjusted x", self.currX + 0.31)
        self.table.putNumber("adjusted y", self.currY + 0.18)
        self.table.putNumber("z", self.currZ)
        self.table.putNumber("rotation", self.currRot)
        self.table.putNumber("desired angle", self.desiredAngle)
        self.table.putNumber("midCamX", self.photonCameraMiddle.robotX)
        self.table.putNumber("midCamY", self.photonCameraMiddle.robotY)
        self.table.putNumber("midCamRot", self.photonCameraMiddle.robotAngle)
        self.xDif = max(self.tagX, self.currX) - min(self.tagX,self.currX)
        self.yDif = max(self.tagY,self.currY) - min(self.tagY, self.currY)
        self.desiredAngle = math.atan(self.yDif/self.xDif)
        self.count = 1
        self.tempX = -1
        self.tempY = -1
        self.tempRot = 0

    def teleopInit(self):
        pass
    def teleopPeriodic(self):
        self.turretMotor.stopMotor()
        if(self.driveCtrlr.getLeftX() > 0.01 or self.driveCtrlr.getLeftX() < -0.01):
            self.turnVoltage = self.driveCtrlr.getLeftX() * 2
            self.turretMotor.setVoltage(self.turnVoltage)
        if(self.driveCtrlr.getAButton()):
            if(self.currRot > self.desiredAngle + 0.01):
                self.turretMotor.setVoltage(-1)
            if(self.currRot < self.desiredAngle - 0.01):
                self.turretMotor.setVoltage(1)