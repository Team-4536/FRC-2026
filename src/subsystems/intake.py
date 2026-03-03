from enum import Enum
from subsystems.motor import RevMotor
from subsystems.robotState import RobotState
from subsystems.subsystem import Subsystem
from wpilib import getTime
import wpilib


class IntakeState(Enum):
    OH_NO = 0
    UP = 1
    GOING_DOWN = 2
    DOWN = 3
    GOING_UP = 4


class Intake(Subsystem):
    # defines all motors that are used in the subsystem
    def __init__(self, frontMotorID: int, backMotorID: int, raiseMotorID: int):
        super().__init__()
        self.intakeMotorManual = RevMotor(deviceID=frontMotorID)
        self.intakeMotorRaise = RevMotor(deviceID=raiseMotorID)
        self.intakeMotorAutomatic = RevMotor(deviceID=backMotorID)
        self.downLimitSwitch = self.intakeMotorRaise.getReverseLimitSwitch()
        self.upLimitSwitch = self.intakeMotorRaise.getForwardLimitSwitch()

        self.AUTOMATIC_MODE = False
        self.intakeRaiseEncoder = self.intakeMotorRaise.getEncoder()
        self.intakeRaiseEncoder.setPosition(0)

        self.state = IntakeState.UP

        self.publishFloat("intake_speed (0 to 1)", 0.7)
        self.publishFloat("reverse_speed (0 to 1)", 0.7)
        self.publishFloat("indexer_speed (0 to 1)", 0.4)

    def phaseInit(self, robotState: RobotState) -> RobotState:
        self.intakeMotorAutomatic.configure(config=RevMotor.INDEXER_MOTOR_CONFIG)
        self.intakeMotorManual.configure(config=RevMotor.INTAKE_MOTOR_CONFIG)
        self.intakeMotorRaise.configure(config=RevMotor.INTAKE_RAISE_CONFIG)

        # these set the speed of the intake motors (negative is forward...):
        self.raiseDownSetpoint = 0.2
        self.raiseUpSetpoint = -0.5
        self.raiseStayUpSetpoint = -0.150

        # default 0
        self.raiseThrottle = 0
        self.manualThrottle = 0
        self.automaticThrottle = 0

        if self.downLimitSwitch:
            self.state = IntakeState.DOWN
        elif self.upLimitSwitch:
            self.state = IntakeState.UP
        else:
            self.state = IntakeState.OH_NO

        robotState.intakePos = False

        return robotState

    def periodic(self, robotState: RobotState) -> RobotState:
        self.motorForwardSetpoint = -max(
            min(self.getFloat("intake_speed (0 to 1)", default=0.0), 1.0), 0
        )
        self.motorReverseSetpoint = max(
            min(self.getFloat("reverse_speed (0 to 1)", default=0.0), 1.0), 0
        )
        self.indexerSetpoint = -max(
            min(self.getFloat("indexer_speed (0 to 1)", default=0.0), 1.0), 0
        )

        if not self.AUTOMATIC_MODE:  # MANUAL MODE!! ITS THE ONLY MODE FOR ME
            # change these values if you need to decrease/increase raise and lowering speed
            if robotState.intakePosYAxis < -0.1:
                self.raiseThrottle = robotState.intakePosYAxis * 0.2  # down
            elif robotState.intakePosYAxis > 0.1:
                self.raiseThrottle = robotState.intakePosYAxis * 0.5  # up
            else:
                self.raiseThrottle = 0  # dead

        elif self.AUTOMATIC_MODE:  # AUTOMATIC MODE

            if self.state == IntakeState.UP:
                self.raiseThrottle = self.raiseStayUpSetpoint
                if robotState.intakePos:
                    self.startTime = wpilib.getTime()
                    self.state = IntakeState.GOING_DOWN

            elif self.state == IntakeState.GOING_DOWN:
                if getTime() - self.startTime < 0.3:
                    self.raiseThrottle = self.raiseDownSetpoint
                else:
                    self.raiseThrottle = 0
                    self.state = IntakeState.DOWN

            elif self.state == IntakeState.DOWN:
                if robotState.intakePos:
                    self.state = IntakeState.GOING_UP

            elif self.state == IntakeState.GOING_UP:
                self.raiseThrottle = self.raiseUpSetpoint

        # DOWN: 23.857    UP: -0.35
        if self.downLimitSwitch.get() or self.intakeRaiseEncoder.getPosition() > 20:
            self.state = IntakeState.DOWN
            if self.raiseThrottle > 0:
                self.intakeMotorRaise.stopMotor()
        elif self.upLimitSwitch.get() or self.intakeRaiseEncoder.getPosition() < 0:
            self.state = IntakeState.UP
            if self.raiseThrottle < 0:
                self.intakeMotorRaise.stopMotor()
            if self.upLimitSwitch.get():
                self.intakeRaiseEncoder.setPosition(-0.35)  # TODO: NOT ACTUAL VALUE
        else:
            self.intakeMotorRaise.setThrottle(self.raiseThrottle)

        self.AUTOMATIC_MODE = robotState.intakeMode  # pyright: ignore

        # initial motor that intakes
        if robotState.initialIntake:
            self.manualThrottle = self.motorForwardSetpoint
        else:
            self.manualThrottle = 0

        # second motor (intakes into subsystem), should eventually be a sensor but is a button rn
        if robotState.intakeIndexer:
            self.automaticThrottle = self.indexerSetpoint
            self.manualThrottle = self.motorForwardSetpoint
        else:
            self.automaticThrottle = 0

        # makes both motors go backwards when something goes wrong
        if robotState.ejectAll > 0.3:
            self.manualThrottle = self.motorReverseSetpoint
            self.automaticThrottle = self.motorReverseSetpoint

        if robotState.intakeEject:
            self.manualThrottle = self.motorReverseSetpoint

        if robotState.indexerEject:
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
