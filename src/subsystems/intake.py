from subsystems.subsystem import Subsystem
from subsystems.motor import RevMotor
from subsystems.robotState import RobotState
from ntcore import NetworkTableInstance


class Intake(Subsystem):
    # TODO make intake less terrible looking

    def __init__(self, FrontMotorID, BackMotorID, RaiseMotorID):
        self.table = NetworkTableInstance.getDefault().getTable("telemetry")
        self.intakeMotorManual = RevMotor(deviceID=FrontMotorID)
        self.intakeMotorManual.configure(config=RevMotor.INTAKE_MOTOR_CONFIG)
        self.intakeMotorAutomatic = RevMotor(deviceID=BackMotorID)
        self.intakeMotorRaise = RevMotor(deviceID=RaiseMotorID)

    def phaseInit(self) -> None:

        self.AUTOMATIC_MODE = False

        # these set the speed of the intake motors:
        self.motorForwardSetpoint = -0.7
        self.motorReverseSetpoint = 0.5

        # default 0
        self.raiseThrottle = 0
        self.manualThrottle = 0
        self.automaticThrottle = 0

    def periodic(self, robotState: RobotState) -> RobotState:

        if not self.AUTOMATIC_MODE:  # MANUAL MODE !!!! THE ONLY MODE FOR ME

            if robotState.intakePosAxis > -0.1:
                self.raiseThrottle = robotState.intakePosAxis * -0.2  # down

            elif robotState.intakePosAxis < 0.1:
                self.raiseThrottle = robotState.intakePosAxis * -0.5  # up

            else:
                self.raiseThrottle = 0  # dead

            self.intakeMotorRaise.setThrottle(self.raiseThrottle)

        if robotState.intakeManualButton:
            self.manualThrottle = self.motorForwardSetpoint
        else:
            self.manualThrottle = 0

        if robotState.intakeSensorTest:
            self.automaticThrottle = self.motorForwardSetpoint
        else:
            self.automaticThrottle = 0

        if robotState.intakeEjectButton:
            self.manualThrottle = self.motorReverseSetpoint
            self.automaticThrottle = self.motorReverseSetpoint

        self.intakeMotorAutomatic.setThrottle(self.automaticThrottle)
        self.intakeMotorManual.setThrottle(self.manualThrottle)

        # TODO get rid of all these table. put them where they should be

        self.table.putNumber("intakeMThrottle", self.manualThrottle)
        self.table.putNumber("intakeAThrottle", self.automaticThrottle)
        self.table.putNumber("intakeRThrottle", self.raiseThrottle)
        self.table.putBoolean("intakeAutoMode", self.AUTOMATIC_MODE)

        return robotState

    def disabled(self):
        self.intakeMotorManual.setThrottle(0)
        self.intakeMotorAutomatic.setThrottle(0)
        self.intakeMotorRaise.setThrottle(0)
        self.manualThrottle = 0
        self.automaticThrottle = 0
        self.raiseThrottle = 0

    def publish(self):
        pass
