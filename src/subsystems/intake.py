from subsystems.subsystem import Subsystem
from subsystems.motor import RevMotor
from subsystems.robotState import RobotState
from ntcore import NetworkTableInstance


class Intake(Subsystem):
    # TODO make intake less terrible looking

    # RADIANSNNSNSNSNNS
    INTAKE_UP = 0
    INTAKE_DOWN = 0.5 * 3.1415926

    def __init__(self, FrontMotorID, BackMotorID, RaiseMotorID):
        self.table = NetworkTableInstance.getDefault().getTable("telemetry")
        self.intakeMotorManual = RevMotor(deviceID=FrontMotorID)
        self.intakeMotorAutomatic = RevMotor(deviceID=BackMotorID)
        self.intakeMotorRaise = RevMotor(deviceID=RaiseMotorID)

    def phaseInit(self) -> None:
        self.intakeVelManual = 0
        self.intakeVelAutomatic = 0
        # TODO MAKE SURE THE RADIANS ON THIS ARE RIGHT!!!!

        self.intakePos = self.INTAKE_UP
        self.intakeMotorRaise.setPosition(self.INTAKE_UP)
        self.intakeSetPosition = self.INTAKE_UP
        # these set the speed of the intake motors:
        self.velSetpoint = 1
        self.ejectSetpoint = -5

        # TODO change this to an actual sensor when you have one
        # self.intakeSensor = sensing boi

    def periodic(self, rs: RobotState) -> RobotState:
        # 4 buttons in subsystem: manualbutton that controls the first motor
        # intakeSensortest that's a fake sensor for now
        # ejectButton that controls eject
        # intakePosButton that controls up or down of intake

        self.intakePos = self.intakeMotorRaise.getEncoder().getPosition()

        if (
            self.INTAKE_DOWN - 0.05 < self.intakePos < self.INTAKE_DOWN + 0.05
        ):  # intake down
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

            if rs.intakePosButton:
                self.intakeSetPosition = self.INTAKE_UP
            else:
                self.intakeSetPosition = self.INTAKE_DOWN

        elif (
            self.INTAKE_UP - 0.05 < self.intakePos < self.INTAKE_UP + 0.05
        ):  # intake up
            self.intakeVelManual = 0
            self.intakeVelAutomatic = 0
            if rs.intakePosButton:
                self.intakeSetPosition = self.INTAKE_DOWN
            else:
                self.intakeSetPosition = self.INTAKE_UP

        self.intakeMotorManual.setVelocity(self.intakeVelManual)
        self.intakeMotorAutomatic.setVelocity(self.intakeVelAutomatic)
        self.intakeMotorRaise.setPosition(self.intakeSetPosition)
        self.intakePos = self.intakeSetPosition

        return rs

    def disabled(self):
        self.intakeMotorManual.setVelocity(0)
        self.intakeMotorAutomatic.setVelocity(0)
        self.intakeVelManual = 0
        self.intakeVelAutomatic = 0

    def publish(self):
        # self.publishFloat("manual velocity", self.intakeVelManual, "Intake")
        # self.publishFloat("automatic velocity", self.intakeVelAutomatic, "Intake")
        # self.publishFloat(
        #     "intake raise encoder",
        #     self.intakeMotorRaise.getEncoder().getPosition(),
        #     "Intake",
        # )
        # self.publishBoolean("intake sensor", self.intakeSensor, "Intake")

        # self.table.putNumber("intakeManualVel", self.intakeVelManual)
        # self.table.putNumber("intakeAutoVel", self.intakeVelAutomatic)
        # self.table.putNumber("intakePos", self.intakePos)
        # self.table.putNumber("intakeSetPosition", self.intakeSetPosition)

        # self.table.putBoolean("intakeEjectButton", rs.intakeEjectButton)
        # self.table.putBoolean("intakeManualButton", rs.intakeManualButton)
        # self.table.putBoolean("intakePosButton", rs.intakePosButton)
        # self.table.putBoolean("intakeSensorTest", rs.intakeSensorTest)
        pass
