import wpilib
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

class Robot(wpilib.TimedRobot):
    def robotInit(self):
        self.driveCtrlr = wpilib.XboxController(0)
        self.table = NetworkTableInstance.getDefault().getTable("telemetry")
        self.manipulatorMotor = SparkMax(3, SparkMax.MotorType.kBrushless)
        self.otherMotor = SparkMax(4, SparkMax.MotorType.kBrushless)
        
    def robotPeriodic(self):
        pass
    def teleopInit(self):
        pass


    def teleopPeriodic(self):
        self.manipulatorMotor.stopMotor()
        self.otherMotor.stopMotor()
        if(self.driveCtrlr.getAButtonPressed()):
            self.manipulatorMotor.setVoltage(3)
            self.otherMotor.setVoltage(-3)