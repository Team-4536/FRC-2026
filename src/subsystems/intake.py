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
        self.manualThrottle = 0
        self.automaticThrottle = 0
        self.AUTOMATIC_MODE = False
        # TODO MAKE SURE THE RADIANS ON THIS ARE RIGHT!!!!
        self.intakePos = self.INTAKE_UP
        self.intakeMotorRaise.setPosition(self.INTAKE_UP)
        self.intakeSetPosition = self.INTAKE_UP
        # these set the speed of the intake motors:
        self.velSetpoint = 0.5
        self.ejectSetpoint = -0.5

        # default 0
        self.raiseThrottle = 0

        # TODO change this to an actual sensor when you have one
        # self.intakeSensor = sensing boi

    def periodic(self, rs: RobotState) -> RobotState:
        # 4 buttons in subsystem: manualbutton that controls the first motor
        # intakeSensortest that's a fake sensor for now
        # ejectButton that controls eject
        # intakePosButton that controls up or down of intake

        if self.AUTOMATIC_MODE:
            self.intakePos = self.intakeMotorRaise.getEncoder().getPosition()

            if (
                self.INTAKE_DOWN - 0.05 < self.intakePos < self.INTAKE_DOWN + 0.05
            ):  # intake down
                if rs.intakeManualButton:
                    self.manualThrottle = self.velSetpoint
                else:
                    self.manualThrottle = 0

                # TODO change this as well when you have an actual sensor
                if rs.intakeSensorTest:
                    self.automaticThrottle = self.velSetpoint
                else:
                    self.automaticThrottle = 0

                # emergency button in case a ball gets stuck
                if rs.intakeEjectButton:
                    self.manualThrottle = self.ejectSetpoint
                    self.automaticThrottle = self.ejectSetpoint

                if rs.intakePosButton:
                    self.intakeSetPosition = self.INTAKE_UP
                else:
                    self.intakeSetPosition = self.INTAKE_DOWN

            elif (
                self.INTAKE_UP - 0.05 < self.intakePos < self.INTAKE_UP + 0.05
            ):  # intake up
                self.manualThrottle = 0
                self.automaticThrottle = 0
                if rs.intakePosButton:
                    self.intakeSetPosition = self.INTAKE_DOWN
                else:
                    self.intakeSetPosition = self.INTAKE_UP

            self.intakeMotorManual.setVelocity(self.manualThrottle)
            self.intakeMotorAutomatic.setVelocity(self.automaticThrottle)
            self.intakeMotorRaise.setPosition(self.intakeSetPosition)
            self.intakePos = self.intakeSetPosition

        if not self.AUTOMATIC_MODE:
            if rs.intakePosAxis > 0.1:
                self.raiseThrottle = rs.intakePosAxis * 0.2  # down
            elif rs.intakePosAxis < -0.1:
                self.raiseThrottle = rs.intakePosAxis * 0.5  # up
            else:
                self.raiseThrottle = 0  # dead

            if rs.intakeManualButton:
                self.manualThrottle = self.velSetpoint
            else:
                self.manualThrottle = 0

            if rs.intakeSensorTest:
                self.automaticThrottle = self.velSetpoint
            else:
                self.automaticThrottle = 0

            if rs.intakeEjectButton:
                self.manualThrottle = self.ejectSetpoint
                self.automaticThrottle = self.ejectSetpoint

            self.intakeMotorAutomatic.setThrottle(self.automaticThrottle)
            self.intakeMotorManual.setThrottle(self.manualThrottle)
            self.intakeMotorRaise.setThrottle(self.raiseThrottle)

        self.table.putNumber("intakeManualVel", self.manualThrottle)
        self.table.putNumber("intakeAutoVel", self.automaticThrottle)
        self.table.putNumber(
            "intakePos", self.intakeMotorRaise.getEncoder().getPosition()
        )

        self.table.putBoolean("intakeEjectButton", rs.intakeEjectButton)
        self.table.putBoolean("intakeManualButton", rs.intakeManualButton)
        self.table.putBoolean("intakePosAxis", rs.intakePosAxis)
        self.table.putBoolean("intakeSensorTest", rs.intakeSensorTest)

        return rs

    def disabled(self):
        self.intakeMotorManual.setVelocity(0)
        self.intakeMotorAutomatic.setVelocity(0)
        self.manualThrottle = 0
        self.automaticThrottle = 0

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
