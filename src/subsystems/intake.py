from subsystems.subsystem import Subsystem
from subsystems.motor import RevMotor
from subsystems.robotState import RobotState
from ntcore import NetworkTableInstance
from enum import Enum
import wpilib


class IntakeState(Enum):
    UP = 0
    DOWN = 1
    GOINGDOWN = 2
    GOINGUP = 3
    MANUAL = 99


class Intake(Subsystem):
    # TODO make intake less terrible looking

    def __init__(self, FrontMotorID, BackMotorID, RaiseMotorID):
        self.table = NetworkTableInstance.getDefault().getTable("telemetry")
        self.intakeMotorManual = RevMotor(deviceID=FrontMotorID)
        self.intakeMotorAutomatic = RevMotor(deviceID=BackMotorID)
        self.intakeMotorRaise = RevMotor(deviceID=RaiseMotorID)

    def phaseInit(self) -> None:

        self.AUTOMATIC_MODE = False

        self.backSensor = True  # wpilib.DigitalInput(17) TODO lowkey fix this channel
        self.forwardSensor = (
            False  # wpilib.DigitalInput(8) TODO lowkey fix this channel
        )

        # these set the speed of the intake motors:
        self.motorForwardSetpoint = -0.7
        self.motorReverseSetpoint = 0.5
        self.raiseUpSetpoint = 0.5
        self.raiseDownSetpoint = -0.5

        # default 0
        self.raiseThrottle = 0
        self.manualThrottle = 0
        self.automaticThrottle = 0

    def periodic(self, rs: RobotState) -> RobotState:
        # 4 buttons in subsystem: manualbutton that controls the first motor
        # intakeSensortest that's a fake sensor for now
        # ejectButton that controls eject
        # intakePosButton that controls up or down of intake

        if self.AUTOMATIC_MODE:  # AUTOMATIC MODE!!!!

            if rs.intakePosButton and self.forwardSensor:
                self.raiseThrottle = self.raiseUpSetpoint
            elif rs.intakePosButton and self.backSensor:
                self.raiseThrottle = self.raiseDownSetpoint

            if (self.raiseThrottle == self.raiseUpSetpoint) and self.backSensor:
                self.raiseThrottle = 0
            if (self.raiseThrottle == self.raiseDownSetpoint) and self.forwardSensor:
                self.raiseThrottle = 0

            if rs.intakeMode:
                self.AUTOMATIC_MODE = False

            self.intakeMotorRaise.setThrottle(self.raiseThrottle)

        elif not self.AUTOMATIC_MODE:  # MANUAL MODE !!!!

            if rs.intakePosAxis > 0.1:
                self.raiseThrottle = rs.intakePosAxis * -0.2  # down
                if self.forwardSensor:
                    self.raiseThrottle = 0
            elif rs.intakePosAxis < -0.1:
                self.raiseThrottle = rs.intakePosAxis * -0.5  # up
                if self.backSensor:
                    self.raiseThrottle = 0
            else:
                self.raiseThrottle = 0  # dead
            if rs.intakeMode:
                self.AUTOMATIC_MODE = False
                self.raiseThrottle = 0

            self.intakeMotorRaise.setThrottle(self.raiseThrottle)

        if self.forwardSensor:
            if rs.intakeManualButton:
                self.manualThrottle = self.motorForwardSetpoint
            else:
                self.manualThrottle = 0

            if rs.intakeSensorTest:
                self.automaticThrottle = self.motorForwardSetpoint
            else:
                self.automaticThrottle = 0

            if rs.intakeEjectButton:
                self.manualThrottle = self.motorReverseSetpoint
                self.automaticThrottle = self.motorReverseSetpoint

            self.intakeMotorAutomatic.setThrottle(self.automaticThrottle)
            self.intakeMotorManual.setThrottle(self.manualThrottle)

        else:
            self.intakeMotorManual.setThrottle(0)
            self.intakeMotorAutomatic.setThrottle(0)
            self.manualThrottle = 0
            self.automaticThrottle = 0

        # TODO get rid of all these table. put them were they should be

        self.table.putNumber("intakeMThrottle", self.manualThrottle)
        self.table.putNumber("intakeAThrottle", self.automaticThrottle)
        self.table.putNumber("intakeRThrottle", self.raiseThrottle)

        self.table.putBoolean("intakePosButton", rs.intakePosButton)
        self.table.putBoolean("intakeModeButton", rs.intakeMode)
        self.table.putBoolean("intakeAutoMode", self.AUTOMATIC_MODE)

        self.table.putBoolean("intakeBackSensor", self.backSensor)
        self.table.putBoolean("intakeForwardSensor", self.forwardSensor)

        return rs

    def disabled(self):
        self.intakeMotorManual.setThrottle(0)
        self.intakeMotorAutomatic.setThrottle(0)
        self.intakeMotorRaise.setThrottle(0)
        self.manualThrottle = 0
        self.automaticThrottle = 0
        self.raiseThrottle = 0

    def publish(self):
        # self.publishFloat("manual velocity", self.manualThrottle, "Intake")
        # self.publishFloat("automatic velocity", self.automaticThrottle, "Intake")
        # self.publishFloat(
        #     "intake raise encoder",
        #     self.intakeMotorRaise.getEncoder().getPosition(),
        #     "Intake",
        # )
        # self.publishBoolean("intake sensor", self.intakeSensor, "Intake")

        pass
