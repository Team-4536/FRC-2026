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


class Intake(Subsystem):
    # defines all motors that are used in the subsystem
    def __init__(self, frontMotorID: int, backMotorID: int, RaiseMotorID: int):
        super().__init__()
        self.intakeMotorManual = RevMotor(deviceID=frontMotorID)
        self.intakeMotorRaise = RevMotor(deviceID=RaiseMotorID)
        self.intakeMotorAutomatic = RevMotor(deviceID=backMotorID)

        self.state = (
            IntakeState.UP
        )  # DO NOT put this in phaseInit. bad things will happen
        self.AUTOMATIC_MODE = False

        self.publishFloat("intake_speed (0 to 1)", 0.2)
        self.publishFloat("reverse_speed (0 to 1)", 0.2)
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

            self.intakeMotorRaise.setThrottle(self.raiseThrottle)
            if robotState.intakeMode:
                self.AUTOMATIC_MODE = False  # pyright: ignore

        if (
            self.AUTOMATIC_MODE
        ):  # AUTOMATIC MODE. !!!! MAKE SURE INTAKE IS UP !!!! BEFORE ENTERING AUTOMATIC MODE.
            if self.state == IntakeState.UP:
                self.raiseThrottle = self.raiseStayUpSetpoint
                if robotState.intakePos == 0:
                    self.startTime = getTime()
                    self.state = IntakeState.GOING_DOWN
            if self.state == IntakeState.GOING_DOWN:
                if getTime() - self.startTime < 0.3:
                    self.raiseThrottle = self.raiseDownSetpoint
                else:
                    self.raiseThrottle = 0
                    self.state = IntakeState.DOWN
            if self.state == IntakeState.DOWN:
                if robotState.intakePos == 180:
                    self.state = IntakeState.GOING_UP
            if self.state == IntakeState.GOING_UP:
                self.raiseThrottle = self.raiseUpSetpoint
                if getTime() - self.startTime < 1:
                    self.state = IntakeState.UP
            self.intakeMotorRaise.setThrottle(self.raiseThrottle)
            if robotState.intakeMode:
                self.AUTOMATIC_MODE = False  # pyright: ignore

        # initial motor that intakes, should be the only one you press a button for
        if robotState.initialIntake:
            self.manualThrottle = self.motorForwardSetpoint
        else:
            self.manualThrottle = 0

        # second motor (intakes into subsystem), should eventually be a sensor but is a button rn
        if robotState.intakeIndexer:
            self.automaticThrottle = self.indexerSetpoint
        else:
            self.automaticThrottle = 0

        # makes both motors go backwards when something goes wrong
        if robotState.ejectAll > 0.3:
            self.manualThrottle = self.motorReverseSetpoint
            self.automaticThrottle = self.motorReverseSetpoint

        if robotState.intakeEject:
            self.manualThrottle = self.motorReverseSetpoint

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
