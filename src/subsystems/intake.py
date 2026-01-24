from subsystems.subsystem import Subsystem
from subsystems.desiredState import DesiredState
from ntcore import NetworkTableInstance
from subsystems.motor import RevMotor


class Intake(Subsystem):
    def __init__(self):
        self.table = NetworkTableInstance.getDefault().getTable("telemetry")
        self.intakeMotorOne = RevMotor(deviceID=10)
        self.intakeMotorTwo = RevMotor(deviceID=9)
        self.intakeVelocityOne = 0
        self.intakeVelocityTwo = 0
        # change to the actual ID v
        # self.intakeSensor = wpilib.AnalogInput(3).getValue()
        self.table.putNumber("intake volts 1", self.intakeVelocityOne)
        self.table.putNumber("intake volts 2", self.intakeVelocityTwo)
        # self.table.putNumber("intake sensor", self.intakeSensor)

    def init(self) -> None:
        pass

    def periodic(self, ds: DesiredState):
        if ds.AButton:
            self.intakeVelocityOne = 1
        else:
            self.intakeVelocityOne = 0

        if ds.BButton:
            self.intakeVelocityTwo = 1
        else:
            self.intakeVelocityTwo = 0

        self.intakeMotorOne.setVelocity(self.intakeVelocityOne)
        self.intakeMotorTwo.setVelocity(self.intakeVelocityTwo)

    def disabled(self):
        self.intakeMotorOne.setVelocity(0)
        self.intakeMotorTwo.setVelocity(0)
        self.intakeVelocityOne = 0
        self.intakeVelocityTwo = 0

    def publish(self):
        pass
