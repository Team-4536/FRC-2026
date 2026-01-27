import wpilib
import wpimath
from subsystems.cameras import photonCameraClass
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
from subsystemManager import SubsystemManager, Subsystems
from subsystems.inputs import Inputs
from subsystems.LEDSignals import LEDSignals
from subsystems.swerveDrive import SwerveDrive
from subsystems.utils import TimeData
from wpilib import TimedRobot



class Robot(wpilib.TimedRobot):
    def robotInit(self):
        self.subsystems: SubsystemManager = SubsystemManager(
            Subsystems(
                inputs=Inputs(),
                ledSignals=LEDSignals(deviceID=0),
                swerveDrive=SwerveDrive(),
                time=TimeData(),
            )
        )
        self.whereToAimRot = wpimath.geometry.Rotation3d
        self.whereToAim = wpimath.geometry.Pose3d(wpimath.geometry.Transform3d(0,0,0))
        
        
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
        self.subsystems.robotPeriodic()
     
       
        
        # elif(self.photonCameraRight.trustworthy):
        #     self.currX = self.photonCameraRight.robotX
        #     self.currY = self.photonCameraRight.robotY
        #     self.currRot = self.photonCameraRight.robotAngle
        # elif(self.photonCameraLeft.trustworthy):
        #     self.currX = self.photonCameraLeft.robotX
        #     self.currY = self.photonCameraLeft.robotY
        #     self.currRot = self.photonCameraLeft.robotAngle
        #     self.currZ = self.photonCameraLeft.robotZ
        # if(self.photonCameraLeft.trustworthy):
        #     self.tempX += self.photonCameraLeft.robotX
        #     self.tempY += self.photonCameraLeft.robotY
        #     self.tempRot += self.photonCameraLeft.robotAngle
        #     self.count += 1
        # if(self.photonCameraMiddle.trustworthy):
        #     self.tempX += self.photonCameraMiddle.robotX
        #     self.tempY += self.photonCameraMiddle.robotY
        #     self.tempRot += self.photonCameraMiddle.robotAngle
        #     self.count += 1
      
        # if(self.tempX >= 0 and self.tempY >= 0):
        #     self.currX = self.tempX / self.count
        #     self.currY = self.tempY / self.count
        #     self.currRot = self.tempRot / self.count
        # self.table.putBoolean("cam1 trustworthy", self.photonCameraRight.trustworthy)
        # self.table.putBoolean("cam2 trustworthy", self.photonCameraLeft.trustworthy)
        
        # self.table.putNumber("x", self.currX)
        # self.table.putNumber("y", self.currY)
        # self.table.putNumber("adjusted x", self.currX + 0.31)
        # self.table.putNumber("adjusted y", self.currY + 0.18)
        # self.table.putNumber("z", self.currZ)
        # self.table.putNumber("rotation", self.currRot)
        # #self.table.putNumber("desired angle", self.desiredAngle)
        
        # self.xDif = max(self.tagX, self.currX) - min(self.tagX,self.currX)
        # self.yDif = max(self.tagY,self.currY) - min(self.tagY, self.currY)
        # self.desiredAngle = math.atan(self.yDif/self.xDif)
        # self.count = 1
        # self.tempX = -1
        # self.tempY = -1
        # self.tempRot = 0

    def teleopInit(self):
        # self.subsystems.init()

        # for s in self.subsystems:
        #     s.init()
        pass
    def teleopPeriodic(self):
        # self.turretMotor.stopMotor()
        # #self.subsystems.teleopPeriodic()
        # if(self.driveCtrlr.getLeftX() > 0.01 or self.driveCtrlr.getLeftX() < -0.01):
        #     self.turnVoltage = self.driveCtrlr.getLeftX() * 2
        #     self.turretMotor.setVoltage(self.turnVoltage)
        # if(self.driveCtrlr.getAButton()):
        #     if(self.currRot > self.desiredAngle + 0.01):
        #         self.turretMotor.setVoltage(-1)
        #     if(self.currRot < self.desiredAngle - 0.01):
        #         self.turretMotor.setVoltage(1)
        pass
    def autonomousInit(self) -> None:
        self.autonomousPeriodic()

    def autonomousPeriodic(self) -> None:
        pass
    def disabledInit(self) -> None:
        self.disabledPeriodic()
        

    def disabledPeriodic(self) -> None:
        #self.subsystems.disabled()
        pass