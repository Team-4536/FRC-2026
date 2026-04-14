from enum import Enum
from subsystems.motor import RevMotor
from subsystems.robotState import RobotState
from subsystems.subsystem import Subsystem
from wpilib import getTime


class IntakeState(Enum):
    OH_NO = 0
    UP = 1
    GOING_DOWN = 2
    DOWN = 3
    GOING_UP = 4
    # when using states for autos, do IntakeState.[insert state], it will act as a simple check,
    # if needed I can make a more robust system


class Intake(Subsystem):
    # defines all motors that are used in the subsystem
    def __init__(self, frontMotorID: int, backMotorID: int, raiseMotorID: int):

        super().__init__()
        self.intakeMotorManual = RevMotor(deviceID=frontMotorID)
        self.intakeMotorRaise = RevMotor(deviceID=raiseMotorID)
        self.intakeMotorAutomatic = RevMotor(deviceID=backMotorID)
        self.downLimitSwitch = self.intakeMotorRaise.getForwardLimitSwitch()
        self.upLimitSwitch = self.intakeMotorRaise.getReverseLimitSwitch()
        # currently limit switches are true by default, false when pressed

        self.automaticMode = False
        self.state = IntakeState.UP
        self.lowerIntake = False
        self.publishFloat("intake_speed (0 to 1)", 0.7)
        self.publishFloat("reverse_speed (0 to 1)", 0.7)
        self.publishFloat("indexer_speed (0 to 1)", 0.5)

    def phaseInit(self, robotState: RobotState) -> None:
        self.intakeMotorAutomatic.configure(config=RevMotor.INDEXER_MOTOR_CONFIG)
        self.intakeMotorManual.configure(config=RevMotor.INTAKE_MOTOR_CONFIG)
        self.intakeMotorRaise.configure(config=RevMotor.INTAKE_RAISE_CONFIG)

        # these set the speed of the intake motors (negative is forward...):
        self.raiseDownSetpoint = 0.3
        self.raiseUpSetpoint = -0.4
        self.raiseStayUpSetpoint = -0.02
        self.downSetpoint = 0

        # default 0
        self.raiseThrottle = 0
        self.manualThrottle = 0
        self.indexerThrottle = 0

        if self.downLimitSwitch:
            self.state = IntakeState.DOWN
        elif self.upLimitSwitch:
            self.state = IntakeState.UP

    def periodic(self, robotState: RobotState) -> None:
        self.robotState = RobotState
        self.motorForwardSetpoint = -0.7
        # -max(
        #     min(self.getFloat("intake_speed (0 to 1)", default=0.0), 1.0), 0
        # )
        self.motorReverseSetpoint = 0.7
        # max(
        #     min(self.getFloat("reverse_speed (0 to 1)", default=0.0), 1.0), 0
        # )
        self.indexerSetpoint = -0.7
        # -max(
        #     min(self.getFloat("indexer_speed (0 to 1)", default=0.0), 1.0), 0
        # )
        self.lowerIntake = robotState.intakePos

        if robotState.intakeModeLeftBumperPressed:
            self.automaticMode = not self.automaticMode

        if not self.automaticMode:  # MANUAL MODE!! ITS THE ONLY MODE FOR ME
            # change these values if you need to decrea se/increase raise and lowering speed
            if robotState.intakePosYAxis < -0.1:
                self.raiseThrottle = robotState.intakePosYAxis * 0.2  # down
            elif robotState.intakePosYAxis > 0.1:
                self.raiseThrottle = robotState.intakePosYAxis * 0.2  # up
            else:
                self.raiseThrottle = 0  # dead

        elif self.automaticMode:  # AUTOMATIC MODE
            if self.state == IntakeState.UP:
                self.raiseThrottle = self.raiseStayUpSetpoint
                if robotState.intakePos:
                    self.startTime = getTime()
                    self.state = IntakeState.GOING_DOWN

            elif self.state == IntakeState.GOING_DOWN:
                if getTime() - self.startTime < 0.5:
                    self.raiseThrottle = self.raiseDownSetpoint
                else:
                    self.raiseThrottle = 0
                    self.state = IntakeState.DOWN

            elif self.state == IntakeState.DOWN:
                self.raiseThrottle = self.downSetpoint
                if robotState.intakePos:
                    self.state = IntakeState.GOING_UP

            elif self.state == IntakeState.GOING_UP:
                self.raiseThrottle = self.raiseUpSetpoint
                if self.upLimitSwitch.get():
                    self.state = IntakeState.UP

        self.intakeMotorRaise.setThrottle(self.raiseThrottle)

        # initial motor that intakes
        if robotState.initialIntake:
            self.manualThrottle = self.motorForwardSetpoint
        else:
            self.manualThrottle = 0

        # self.publishFloat("mannual_throttle", self.manualThrottle)

        if robotState.intakeIndexer:
            self.indexerThrottle = self.indexerSetpoint
            self.manualThrottle = self.motorForwardSetpoint
        else:
            self.indexerThrottle = 0

        # makes both motors go backwards when something goes wrong
        if robotState.ejectAll > 0.3:
            self.manualThrottle = self.motorReverseSetpoint
            self.indexerThrottle = self.motorReverseSetpoint

        if robotState.intakeEject:
            self.manualThrottle = self.motorReverseSetpoint

        if robotState.indexerEject:
            self.indexerThrottle = self.motorReverseSetpoint

        self.intakeMotorAutomatic.setThrottle(self.indexerThrottle * 7 / 4)
        self.intakeMotorManual.setThrottle(self.manualThrottle * 1.5)

    # values when the robot is disabled
    def disabled(self):
        self.intakeMotorManual.setThrottle(0)
        self.intakeMotorAutomatic.setThrottle(0)
        self.intakeMotorRaise.setThrottle(0)
        self.manualThrottle = 0
        self.indexerThrottle = 0
        self.raiseThrottle = 0

    def publish(self):
        self.publishFloat("intakeMThrottle", self.manualThrottle)
        self.publishFloat("intakeAThrottle", self.indexerThrottle)
        self.publishFloat("intakeRThrottle", self.raiseThrottle)
        self.publishBoolean("intakeAutomaticMode", self.automaticMode)
        self.publishInteger("intakeState", self.state.value)
        self.publishBoolean("intakePos", self.lowerIntake)
        self.publishBoolean("upLimitSwitch", self.upLimitSwitch.get())
        self.publishBoolean("downLimitSwitch", self.downLimitSwitch.get())
        # self.publishBoolean("leftBumper", self.robotState.intakeModeLeftBumperPressed)
