from subsystemManager import SubsystemManager
from subsystems.inputs import Inputs
from subsystems.subsystemPractice import Climber
from subsystems.LEDSignals import LEDSignals
from subsystems.subsystemExample import SubsystemExample
from subsystems.motor import RevMotor
from subsystems.swerveDrive import SwerveDrive
from subsystems.utils import TimeData
from wpilib import TimedRobot


class Robot(TimedRobot):
    def init(self) -> None:
        pass

    def robotInit(self) -> None:
        self.subsystems: SubsystemManager = SubsystemManager(
            inputs=Inputs(),
            ledSignals=LEDSignals(deviceID=0),
            swerveDrive=SwerveDrive(),
            time=TimeData(),
            subsystemExample=SubsystemExample(),
            subsystemClimber=Climber(),
        )

        self.init()

    def robotPeriodic(self) -> None:
        self.subsystems.time.periodic(self.subsystems.desiredState)

    def autonomousInit(self) -> None:
        self.init()

    def autonomousPeriodic(self) -> None:
        pass

    def teleopInit(self) -> None:
        self.init()
        self.subsystems.init()

    def teleopPeriodic(self) -> None:
        self.subsystems.periodic()

    def teleopExit(self) -> None:
        self.disabledInit()

    def disabledInit(self) -> None:
        self.disabledPeriodic()

    def disabledPeriodic(self) -> None:
        self.subsystems.disable()
