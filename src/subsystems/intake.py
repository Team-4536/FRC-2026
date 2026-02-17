from subsystems.subsystem import Subsystem
from subsystems.motor import RevMotor
from subsystems.robotState import RobotState
from ntcore import NetworkTableInstance
from enum import Enum
import wpilib


class IntakeState(Enum):
    OH_NO = 0
    UP = 1
    GOING_DOWN = 2
    DOWN = 3
    GOING_UP = 4


class Intake(Subsystem):

    # defines all motors that are used in the subsystem
    def __init__(self, FrontMotorID, BackMotorID, RaiseMotorID):
        super().__init__()
        self.table = NetworkTableInstance.getDefault().getTable("telemetry")
        self.intakeMotorManual = RevMotor(deviceID=FrontMotorID)
        self.intakeMotorManual.configure(config=RevMotor.INTAKE_MOTOR_CONFIG)
        self.intakeMotorAutomatic = RevMotor(deviceID=BackMotorID)
        self.intakeMotorRaise = RevMotor(deviceID=RaiseMotorID)
        self.intakeMotorRaise.configure(config=RevMotor.INTAKE_RAISE_CONFIG)

        self.state = IntakeState.OH_NO
        self.AUTOMATIC_MODE = False

        self.forwardLimitSwitch = False
        self.backLimitSwitch = True

    def phaseInit(self, robotState: RobotState) -> None:

        # these set the speed of the intake motors:
        self.motorForwardSetpoint = -0.7
        self.motorReverseSetpoint = 0.5
        self.raiseDownSetpoint = 0.2
        self.raiseUpSetpoint = -0.5

        # default 0
        self.raiseThrottle = 0
        self.manualThrottle = 0
        self.automaticThrottle = 0

        if self.backLimitSwitch and not self.forwardLimitSwitch:
            self.state = IntakeState.UP
        elif self.forwardLimitSwitch and not self.backLimitSwitch:
            self.state = IntakeState.DOWN
        else:
            self.state = IntakeState.OH_NO

    def periodic(self, robotState: RobotState) -> RobotState:

        if not self.AUTOMATIC_MODE:  # MANUAL MODE!!!! ITS THE ONLY MODE FOR ME
            if robotState.intakePosYAxis < -0.1:
                self.raiseThrottle = robotState.intakePosYAxis * -0.2  # down
            elif robotState.intakePosYAxis > 0.1:
                self.raiseThrottle = robotState.intakePosYAxis * -0.5  # up
            else:
                self.raiseThrottle = 0  # dead

            self.intakeMotorRaise.setThrottle(self.raiseThrottle)
            if robotState.intakeMode:
                self.AUTOMATIC_MODE = True

        if self.AUTOMATIC_MODE:  # AUTOMATIC MODE.
            if self.state == IntakeState.UP:
                if robotState.intakePos:
                    self.startTime = wpilib.getTime()
                    self.state = IntakeState.GOING_DOWN
            if self.state == IntakeState.GOING_DOWN:
                if wpilib.getTime() - self.startTime < 0.3:
                    self.raiseThrottle = self.raiseDownSetpoint
                else:
                    self.raiseThrottle = 0
                    self.state = IntakeState.DOWN
            if self.state == IntakeState.DOWN:
                if robotState.intakePos:
                    self.state = IntakeState.GOING_UP
            if self.state == IntakeState.GOING_UP:
                self.raiseThrottle = self.raiseUpSetpoint
                if self.backLimitSwitch:
                    self.state = IntakeState.UP
            self.intakeMotorRaise.setThrottle(self.raiseThrottle)
            if robotState.intakeMode:
                self.AUTOMATIC_MODE = False

        # initial motor that intakes, should be the only one you press a button for
        if robotState.initialIntake:
            self.manualThrottle = self.motorForwardSetpoint
        else:
            self.manualThrottle = 0

        # second motor (intakes into susbsystem), should eventually be a sensor but is a button rn
        if robotState.intakeSensorTest:
            self.automaticThrottle = self.motorForwardSetpoint
        else:
            self.automaticThrottle = 0

        # makes both motors go backwards when something goes wrong
        if robotState.intakeEject:
            self.manualThrottle = self.motorReverseSetpoint
            self.automaticThrottle = self.motorReverseSetpoint

        self.intakeMotorAutomatic.setThrottle(self.automaticThrottle)
        self.intakeMotorManual.setThrottle(self.manualThrottle)

        return robotState

    # values when the robot is disabled
    def disabled(self):
        self.intakeMotorManual.setThrottle(0)
        self.intakeMotorAutomatic.setThrottle(0)
        self.intakeMotorRaise.setThrottle(0)
        self.manualThrottle = 0
        self.automaticThrottle = 0
        self.raiseThrottle = 0

    # everything for tables and sims except it crashes
    def publish(self):
        self.publishFloat("intakeMThrottle", self.manualThrottle)
        self.publishFloat("intakeAThrottle", self.automaticThrottle)
        self.publishFloat("intakeRThrottle", self.raiseThrottle)
        self.publishBoolean("intakeAutomaticMode", self.AUTOMATIC_MODE)
        self.publishString("intakeState", self.state.name)
        self.publishBoolean("forwardLimitSwitch", self.forwardLimitSwitch)
        self.publishBoolean("backLimitSwitch", self.backLimitSwitch)
