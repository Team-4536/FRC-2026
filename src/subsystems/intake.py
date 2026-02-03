from subsystems.subsystem import Subsystem
from subsystems.motor import RevMotor
from subsystems.robotState import RobotState


class Intake(Subsystem):
    def __init__(self, FrontMotorID, BackMotorID):
        self.intakeMotorManual = RevMotor(deviceID=FrontMotorID)
        self.intakeMotorAutomatic = RevMotor(deviceID=BackMotorID)

    def init(self) -> None:
        self.intakeVelManual = 0
        self.intakeVelAutomatic = 0
        self.velSetpoint = 1
        self.ejectSetpoint = -5
        # TODO change this to an actual sensor when you have one
        # self.intakeSensor = sensing boi

    def periodic(self, rs: RobotState) -> RobotState:
        if rs.intakeManualButton:
            self.intakeVelManual = self.velSetpoint
        else:
            self.intakeVelManual = 0

        # TODO change this as well when you have an actual sensor
        if rs.intakeSensorTest:
            self.intakeVelAutomatic = self.velSetpoint
        else:
            self.intakeVelAutomatic = 0

        # emergency button in case a ball gets stuck
        if rs.intakeEjectButton:
            self.intakeVelManual = self.ejectSetpoint
            self.intakeVelAutomatic = self.ejectSetpoint
        else:
            self.intakeVelAutomatic = 0
            self.intakeVelManual = 0

        self.intakeMotorManual.setVelocity(self.intakeVelManual)
        self.intakeMotorAutomatic.setVelocity(self.intakeVelAutomatic)

        return rs

    def disabled(self):
        self.intakeMotorManual.setVelocity(0)
        self.intakeMotorAutomatic.setVelocity(0)
        self.intakeVelManual = 0
        self.intakeVelAutomatic = 0

    def publish(self):
        #  TODO check to make sure you did this right
        self.publishFloat("manual velocity", self.intakeVelManual, "Intake")
        self.publishFloat("automatic velocity", self.intakeVelAutomatic, "Intake")
        # self.publishBoolean("intake sensor", self.intakeSensor, "Intake")
